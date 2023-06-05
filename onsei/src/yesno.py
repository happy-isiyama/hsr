

import os
import pyaudio
import pocketsphinx as ps

# PocketSphinxの設定
config = ps.Decoder.default_config()
config.set_string('-hmm', '/path/to/acoustic/model')  # 音響モデルのパス
config.set_string('-dict', '/path/to/dictionary')  # 辞書のパス
config.set_string('-lm', '/path/to/language/model')  # 言語モデルのパス
decoder = ps.Decoder(config)

# マイクからの音声を認識する関数
def recognize_speech():
    # PyAudioの設定
    CHUNK = 1024
    FORMAT = pyaudio.paInt16
    CHANNELS = 1
    RATE = 16000
    RECORD_SECONDS = 5  # 認識する音声の長さ（秒）
    
    p = pyaudio.PyAudio()
    
    # マイクからの音声をストリームで読み取り、PocketSphinxで認識
    with p.open(format=FORMAT, channels=CHANNELS, rate=RATE, input=True, frames_per_buffer=CHUNK) as stream:
        stream.start_stream()

        for i in range(0, int(RATE / CHUNK * RECORD_SECONDS)):
            buf = stream.read(CHUNK, exception_on_overflow=False)
            decoder.process_raw(buf, False, False)
        
        decoder.end_utt()
        
        # 認識結果を取得して返す
        hypothesis = decoder.hyp()
        if hypothesis is not None:
            return hypothesis.hypstr
        else:
            return ""

recognize_speech()
