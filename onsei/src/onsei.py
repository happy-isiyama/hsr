import speech_recognition as sr
import spacy

def recognize_speech_offline(audio_file):
    r = sr.Recognizer()
    with sr.AudioFile(audio_file) as source:
        audio = r.record(source)  # 音声を読み取る

    try:
        text = r.recognize_google(audio, language='ja-JP')  # 音声をテキストに変換
        return text
    except sr.UnknownValueError:
        print("Google Speech Recognition could not understand audio")
        return ""
    except sr.RequestError as e:
        print("Could not request results from Google Speech Recognition service; {0}".format(e))
        return ""

def process_text_offline(text):
    nlp = spacy.load("ja_core_news_sm")  # SpaCyモデルを読み込む
    doc = nlp(text)  # テキストを処理
    return doc

def generate_action_offline(doc):
    # 解析結果から必要な情報を取得して、適切な行動を生成する
    # ここでは、単語の数が5以下の場合は「分かりません」と返す
    if len(doc) <= 5:
        return "分かりません"
    else:
        return "行動を起こします"
