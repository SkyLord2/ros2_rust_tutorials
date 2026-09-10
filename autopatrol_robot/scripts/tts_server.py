#!/usr/bin/env python3
"""提供基于 sherpa-onnx 的离线语音合成服务。

节点接收 SpeechText 请求，依次完成文本规范化、离线推理、WAV 写入和播放。
服务只在播放器退出后返回，使巡检控制器能够等待整段语音播报完成。

模型在启动时加载一次；所有请求共用模型实例和固定输出文件。
互斥锁会串行处理并发请求，防止推理引擎并发访问及音频文件互相覆盖。

只有模型推理、文件写入和播放器执行全部成功时，服务才返回成功。
成功消息包含 WAV 路径，失败消息包含异常原因。

模型目录
--------

目录必须包含 model.onnx、tokens.txt 和 lexicon.txt。
日期、数字、电话及多音字规则文件属于可选资源，仅传入实际存在的文件。
dict 子目录通过 dict_dir 提供中文分词资源；该模型不能把根目录作为
通用 data_dir，否则可能导致文本解析异常。

音频处理
--------

推理结果是单声道浮点波形。写入 16 位 PCM 前需要替换非有限值，
并在峰值超过 1.0 时归一化，防止转换时溢出和爆音。
音频末尾追加可配置静音，避免桌面音频设备关闭时截断最后一个音节。

没有句末标点时自动补中文句号，使 MeloTTS 生成完整的句尾韵律。
max_num_sentences 只控制推理分段，不用于截断请求文本。

播放器选择
----------

自动模式在 WSLg 的 PulseAudio 环境优先选择 ffplay，普通 Linux 优先尝试
aplay，并以 ffplay 作为后备。显式命令使用 shlex 拆分且不经过 shell，
生成的 WAV 路径作为最后一个参数追加，避免路径内容被解释为命令。

主要参数
--------

- model_dir：模型资源目录；
- num_threads：推理线程数；
- speaker_id：模型说话人编号；
- speed：语速，允许范围为 0.1 到 3.0；
- silence_after：末尾静音秒数，允许范围为 0 到 5；
- audio_dir：生成音频的保存目录；
- audio_player：自动选择或显式播放器命令；
- service_name：对外提供的 ROS 服务名称。

故障定位
--------

若 WAV 内容完整但没有声音，应单独检查播放器和系统音频设备。
若 WAV 本身过短或静音，应检查输入文本、语速、说话人和模型兼容性。
若只截断最后音节，应适当增加 silence_after，而不是改变语速。
固定输出文件会被后续请求覆盖，既便于诊断，也避免循环巡检持续占用磁盘。
"""

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
    """使用单个模型实例串行处理离线语音请求。"""

    def __init__(self):
        """声明参数、加载模型并创建语音服务。"""
        super().__init__('sherpa_onnx_tts_server')
        # 模型参数用于创建推理器或控制单次合成。
        self.declare_parameter('model_dir', '')
        self.declare_parameter('num_threads', 2)
        self.declare_parameter('speaker_id', 0)
        self.declare_parameter('speed', 0.75)
        self.declare_parameter('silence_after', 0.35)
        # 输出参数控制音频保存及系统播放器。
        self.declare_parameter('audio_dir', '/tmp/autopatrol_tts')
        self.declare_parameter('audio_player', 'auto')
        self.declare_parameter('service_name', '/speech_text')
        # 推理器和固定 WAV 文件是所有请求共享的可变资源。
        self._lock = threading.Lock()
        self._tts = None
        self._setup_error = ''
        # 先加载模型再创建服务，避免首个请求与初始化竞争。
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
        """将播放器配置解析为不经过 shell 的参数列表。"""
        # 显式配置优先于自动探测结果。
        if configured.strip().lower() != 'auto':
            command = shlex.split(configured)
            if not command:
                raise ValueError('audio_player 不能为空')
            return command
        # WSLg 通过兼容 PulseAudio 的套接字提供宿主机音频。
        if os.environ.get('PULSE_SERVER') and shutil.which('ffplay'):
            return ['ffplay', '-nodisp', '-autoexit', '-loglevel', 'error']
        # 普通 Linux 桌面通常提供默认 ALSA 设备。
        if shutil.which('aplay'):
            return ['aplay', '-q']
        # 没有 ALSA 时再尝试由 SDL 自动选择后端的 ffplay。
        if shutil.which('ffplay'):
            return ['ffplay', '-nodisp', '-autoexit', '-loglevel', 'error']
        raise RuntimeError('找不到可用的音频播放器（aplay/ffplay）')

    def _load_model(self):
        """校验模型资源并创建离线语音合成器。"""
        model_dir = pathlib.Path(self.get_parameter('model_dir').value)
        required = ['model.onnx', 'tokens.txt', 'lexicon.txt']
        if not model_dir or not all(
                (model_dir / name).is_file() for name in required):
            self._setup_error = (
                'model_dir 缺少 model.onnx/tokens.txt/lexicon.txt')
            return
        try:
            import sherpa_onnx
            # 仅拼接实际存在的可选文本规范化规则。
            rule_fsts = ','.join(
                str(model_dir / name) for name in
                ('date.fst', 'number.fst', 'phone.fst', 'new_heteronym.fst')
                if (model_dir / name).is_file()  # 增加存在性判断
            )

            # MeloTTS 使用专用的结巴分词字典目录处理中文。
            dict_dir = model_dir / 'dict'
            dict_dir_str = str(dict_dir) if dict_dir.is_dir() else ""
            
            # 不能给 MeloTTS 传入 data_dir，否则会触发引擎文本解析器错乱，只传 dict_dir
            # 绑定模型文件，实际 ONNX 初始化由 OfflineTts 完成。
            vits = sherpa_onnx.OfflineTtsVitsModelConfig(
                model=str(model_dir / 'model.onnx'),
                lexicon=str(model_dir / 'lexicon.txt'),
                tokens=str(model_dir / 'tokens.txt'),
                dict_dir=dict_dir_str
            )
            # 显式使用 CPU，保证不同开发环境下的兼容性。
            model = sherpa_onnx.OfflineTtsModelConfig(
                vits=vits,
                num_threads=int(self.get_parameter('num_threads').value),
                provider='cpu')
            # 每批允许处理多个短句，并保留句间自然停顿。
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
        """生成、保存并同步播放一条语音请求。"""
        self.get_logger().info('收到语音请求: %s' % request.text)
        # 模型加载失败时向每个调用方返回原始错误。
        if self._tts is None:
            response.success = False
            response.message = self._setup_error or 'TTS 未初始化'
            return response
        # 空白文本无法生成有效的模型标记。
        text = request.text.strip()
        if not text:
            response.success = False
            response.message = 'text 不能为空'
            return response
        # 将推理、固定路径写入和播放作为一个完整互斥事务。
        with self._lock:
            try:
                # 缺少句末标点会使 MeloTTS 输出明显偏短，因此补充句号。
                if text[-1] not in '。！？!?；;。':
                    text += '。'
                # 参数可能在节点运行期间改变，因此每次请求都校验范围。
                speed = float(self.get_parameter('speed').value)
                if not 0.1 <= speed <= 3.0:
                    raise ValueError('speed 必须在 0.1 到 3.0 之间')
                self.get_logger().info('正在生成语音: %s，(语速=%.2f)' % (text, speed))
                # 离线推理一次返回完整波形。
                audio = self._tts.generate(
                    text,
                    sid=int(self.get_parameter('speaker_id').value),
                    speed=speed)
                # WAV 头保留模型原生采样率。
                sample_rate = int(audio.sample_rate)

                # 提取浮点波形并进行防御性去噪
                samples = np.array(audio.samples, dtype=np.float32)
                samples = np.nan_to_num(samples)  # 防止模型异常输出 NaN/Inf 导致波形损坏

                # 幅值归一化，防止削峰爆音
                # 只修正真正溢出的峰值，避免无条件放大底噪。
                max_amp = np.max(np.abs(samples))
                if max_amp > 1.0:
                    samples = samples / max_amp
                
                # 限制尾部静音范围，避免错误参数造成过量内存分配。
                silence_after = float(self.get_parameter('silence_after').value)
                if not 0.0 <= silence_after <= 5.0:
                    raise ValueError('silence_after 必须在 0 到 5 秒之间')
                
                # 追加尾部静音
                if silence_after > 0:
                    silence_samples = np.zeros(int(sample_rate * silence_after), dtype=np.float32)
                    samples = np.concatenate((samples, silence_samples))

                # 收到请求后再创建目录，避免输出卷未挂载时阻止节点启动。
                output_dir = pathlib.Path(
                    self.get_parameter('audio_dir').value)
                output_dir.mkdir(parents=True, exist_ok=True)
                output = output_dir / 'patrol_speech.wav'
                # 退出上下文后 RIFF 长度才写完整，随后才能启动播放器。
                with wave.open(str(output), 'wb') as wav:
                    wav.setnchannels(1)
                    wav.setsampwidth(2)
                    wav.setframerate(sample_rate)
                    pcm = (samples * 32767.0).astype(np.int16).tobytes()
                    wav.writeframes(pcm)
                duration = len(samples) / sample_rate
                self.get_logger().info(
                    '音频已生成: %.2f 秒（语速=%.2f）' % (duration, speed))
                # 音频路径作为独立参数传入，目录中的空格不会改变命令含义。
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
    """初始化 ROS、运行语音节点并在退出时释放上下文。"""
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
