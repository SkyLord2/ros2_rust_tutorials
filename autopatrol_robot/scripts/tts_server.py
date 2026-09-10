#!/usr/bin/env python3
import os
import pathlib
import shlex
import shutil
import subprocess
import threading
import wave
import numpy as np

import rclpy
from rclpy.node import Node

from autopatrol_robot.srv import SpeechText


class TtsServer(Node):
    def __init__(self):
        super().__init__('sherpa_onnx_tts_server')
        self.declare_parameter('model_dir', '')
        self.declare_parameter('num_threads', 2)
        self.declare_parameter('speaker_id', 0)
        self.declare_parameter('speed', 0.75)
        self.declare_parameter('silence_after', 0.35)
        self.declare_parameter('audio_dir', '/tmp/autopatrol_tts')
        self.declare_parameter('audio_player', 'auto')
        self.declare_parameter('service_name', '/speech_text')
        self._lock = threading.Lock()
        self._tts = None
        self._setup_error = ''
        self._load_model()
        service_name = str(self.get_parameter('service_name').value)
        self._service = self.create_service(
            SpeechText, service_name, self._speak)
        player = str(self.get_parameter('audio_player').value)
        self._player_command = self._resolve_player(player)
        if self._tts is not None:
            self.get_logger().info(
                'TTS 服务已就绪: %s, 播放器=%s' %
                (service_name, ' '.join(self._player_command)))
        else:
            self.get_logger().error(
                'TTS 服务已启动但模型不可用: %s' % self._setup_error)

    @staticmethod
    def _resolve_player(configured):
        if configured.strip().lower() != 'auto':
            command = shlex.split(configured)
            if not command:
                raise ValueError('audio_player 不能为空')
            return command
        if os.environ.get('PULSE_SERVER') and shutil.which('ffplay'):
            return ['ffplay', '-nodisp', '-autoexit', '-loglevel', 'error']
        if shutil.which('aplay'):
            return ['aplay', '-q']
        if shutil.which('ffplay'):
            return ['ffplay', '-nodisp', '-autoexit', '-loglevel', 'error']
        raise RuntimeError('找不到可用的音频播放器（aplay/ffplay）')

    def _load_model(self):
        model_dir = pathlib.Path(self.get_parameter('model_dir').value)
        required = ['model.onnx', 'tokens.txt', 'lexicon.txt']
        if not model_dir or not all(
                (model_dir / name).is_file() for name in required):
            self._setup_error = (
                'model_dir 缺少 model.onnx/tokens.txt/lexicon.txt')
            return
        try:
            import sherpa_onnx
            rule_fsts = ','.join(
                str(model_dir / name) for name in
                ('date.fst', 'number.fst', 'phone.fst', 'new_heteronym.fst')
                if (model_dir / name).is_file()  # 增加存在性判断
            )

            dict_dir = model_dir / 'dict'
            dict_dir_str = str(dict_dir) if dict_dir.is_dir() else ""
            
            # 不能给 MeloTTS 传入 data_dir，否则会触发引擎文本解析器错乱，只传 dict_dir
            vits = sherpa_onnx.OfflineTtsVitsModelConfig(
                model=str(model_dir / 'model.onnx'),
                lexicon=str(model_dir / 'lexicon.txt'),
                tokens=str(model_dir / 'tokens.txt'),
                dict_dir=dict_dir_str
            )
            model = sherpa_onnx.OfflineTtsModelConfig(
                vits=vits,
                num_threads=int(self.get_parameter('num_threads').value),
                provider='cpu')
            config = sherpa_onnx.OfflineTtsConfig(
                model=model, 
                rule_fsts=rule_fsts, 
                max_num_sentences=5,
                silence_scale=0.5
            )
            self._tts = sherpa_onnx.OfflineTts(config)
            self.get_logger().info('已加载 sherpa_onnx 模型: %s' % model_dir)
        except Exception as exc:
            self._setup_error = f'sherpa_onnx 模型加载失败: {exc}'
            self.get_logger().error(self._setup_error)

    def _speak(self, request, response):
        self.get_logger().info('收到语音请求: %s' % request.text)
        if self._tts is None:
            response.success = False
            response.message = self._setup_error or 'TTS 未初始化'
            return response
        text = request.text.strip()
        if not text:
            response.success = False
            response.message = 'text 不能为空'
            return response
        with self._lock:
            try:
                # MeloTTS produces noticeably shorter audio when the text has
                # no sentence terminator. Keep the requested words unchanged,
                # but give the model a natural end-of-sentence boundary.
                if text[-1] not in '。！？!?；;。':
                    text += '。'
                speed = float(self.get_parameter('speed').value)
                if not 0.1 <= speed <= 3.0:
                    raise ValueError('speed 必须在 0.1 到 3.0 之间')
                self.get_logger().info('正在生成语音: %s，(语速=%.2f)' % (text, speed))
                audio = self._tts.generate(
                    text,
                    sid=int(self.get_parameter('speaker_id').value),
                    speed=speed)
                sample_rate = int(audio.sample_rate)

                # 提取浮点波形并进行防御性去噪
                samples = np.array(audio.samples, dtype=np.float32)
                samples = np.nan_to_num(samples)  # 防止模型异常输出 NaN/Inf 导致波形损坏

                # 幅值归一化，防止削峰爆音
                max_amp = np.max(np.abs(samples))
                if max_amp > 1.0:
                    samples = samples / max_amp
                
                silence_after = float(self.get_parameter('silence_after').value)
                if not 0.0 <= silence_after <= 5.0:
                    raise ValueError('silence_after 必须在 0 到 5 秒之间')
                
                # 追加尾部静音
                if silence_after > 0:
                    silence_samples = np.zeros(int(sample_rate * silence_after), dtype=np.float32)
                    samples = np.concatenate((samples, silence_samples))

                output_dir = pathlib.Path(
                    self.get_parameter('audio_dir').value)
                output_dir.mkdir(parents=True, exist_ok=True)
                output = output_dir / 'patrol_speech.wav'
                with wave.open(str(output), 'wb') as wav:
                    wav.setnchannels(1)
                    wav.setsampwidth(2)
                    wav.setframerate(sample_rate)
                    pcm = (samples * 32767.0).astype(np.int16).tobytes()
                    wav.writeframes(pcm)
                duration = len(samples) / sample_rate
                self.get_logger().info(
                    '音频已生成: %.2f 秒（语速=%.2f）' % (duration, speed))
                command = self._player_command + [str(output)]
                subprocess.run(command, check=True, timeout=30)
                response.success = True
                response.message = str(output)
            except Exception as exc:
                response.success = False
                response.message = str(exc)
                self.get_logger().error('语音播报失败: %s', exc)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = TtsServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
