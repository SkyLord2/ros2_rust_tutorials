#!/usr/bin/env python3
import pathlib
import shlex
import subprocess
import threading
import wave

import rclpy
from rclpy.node import Node

from autopatrol_robot.srv import SpeechText


class TtsServer(Node):
    def __init__(self):
        super().__init__('sherpa_onnx_tts_server')
        self.declare_parameter('model_dir', '')
        self.declare_parameter('num_threads', 2)
        self.declare_parameter('speaker_id', 0)
        self.declare_parameter('speed', 1.0)
        self.declare_parameter('audio_dir', '/tmp/autopatrol_tts')
        self.declare_parameter('audio_player', 'aplay -q')
        self._lock = threading.Lock()
        self._tts = None
        self._setup_error = ''
        self._load_model()
        self._service = self.create_service(
            SpeechText, '/speech_text', self._speak)

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
                ('date.fst', 'number.fst', 'phone.fst', 'new_heteronym.fst'))
            vits = sherpa_onnx.OfflineTtsVitsModelConfig(
                model=str(model_dir / 'model.onnx'),
                lexicon=str(model_dir / 'lexicon.txt'),
                tokens=str(model_dir / 'tokens.txt'),
                data_dir=str(model_dir),
                dict_dir=str(model_dir / 'dict'),
                rule_fsts=rule_fsts)
            model = sherpa_onnx.OfflineTtsModelConfig(
                vits=vits,
                num_threads=int(self.get_parameter('num_threads').value),
                provider='cpu')
            config = sherpa_onnx.OfflineTtsConfig(
                model=model, max_num_sentences=1)
            self._tts = sherpa_onnx.OfflineTts(config)
        except Exception as exc:
            self._setup_error = f'sherpa_onnx 模型加载失败: {exc}'
            self.get_logger().error(self._setup_error)

    def _speak(self, request, response):
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
                audio = self._tts.generate(
                    text,
                    sid=int(self.get_parameter('speaker_id').value),
                    speed=float(self.get_parameter('speed').value))
                output_dir = pathlib.Path(
                    self.get_parameter('audio_dir').value)
                output_dir.mkdir(parents=True, exist_ok=True)
                output = output_dir / 'patrol_speech.wav'
                with wave.open(str(output), 'wb') as wav:
                    wav.setnchannels(1)
                    wav.setsampwidth(2)
                    wav.setframerate(audio.sample_rate)
                    samples = [max(-1.0, min(1.0, float(x)))
                               for x in audio.samples]
                    pcm = b''.join(
                        int(x * 32767).to_bytes(2, 'little', signed=True)
                        for x in samples)
                    wav.writeframes(pcm)
                command = shlex.split(
                    str(self.get_parameter('audio_player').value)
                ) + [str(output)]
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
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
