import pyaudio
import numpy as np

print("test")
def record_audio():
    CHUNK = 1024
    RATE = 44100
    RECORD_SECONDS = 3

    audio = pyaudio.PyAudio()

    # オーディオストリームを開始
    stream = audio.open(format=pyaudio.paInt16,
                        channels=1,
                        rate=RATE,
                        input=True,
                        frames_per_buffer=CHUNK)

    print("録音開始")

    frames = []
    for i in range(0, int(RATE / CHUNK * RECORD_SECONDS)):
        data = stream.read(CHUNK)
        frames.append(np.frombuffer(data, dtype="int16"))

    # オーディオストリームを停止
    stream.stop_stream()
    stream.close()
    audio.terminate()

    print("録音終了")

    # 録音した音声データをndarrayに変換
    return np.concatenate(frames, axis=None)

record_audio()
