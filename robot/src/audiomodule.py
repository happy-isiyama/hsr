import rospy
import sys
from sensor_msgs.msg import Image
from std_msgs.msg import String

sys.path.append("/home/demulab/dspl/src/hsr/whisper/src")
import pyaudio
import wave
import whisper

import hsrb_interface
from hsrb_interface import geometry
from hsrb_interface import Robot

robot = hsrb_interface.Robot()
tts = robot.get("default_tts")

model = whisper.load_model("medium")

class YesNo():
     def __init__():
         print("voice yes or no")
         #model = whisper.load_model("small")

     def voiceyn():
         #model = whisper.load_model("small")
         CHUNK = 1024
         FORMAT = pyaudio.paInt16
         CHANNELS = 1
         RATE = 44100
         RECORD_SECONDS = 5
         WAVE_OUTPUT_FILENAME = "output.wav"

         p = pyaudio.PyAudio()

         stream = p.open(format = FORMAT,
                    channels = CHANNELS,
                    rate = RATE,
                    input = True,
                    #output = True,
                    #input_device_index = 2,
                    #output_device_index = 1,
                    frames_per_buffer = CHUNK)

         print("recording")
         tts.say("注文を取りに行ってもいいですか")
         frames = []

         for i in range(0,int(RATE / CHUNK * RECORD_SECONDS)):
             d = stream.read(CHUNK)
             frames.append(d)

         print("done recording")

         stream.stop_stream()
         stream.close()
         p.terminate()

         wf = wave.open(WAVE_OUTPUT_FILENAME,'wb')
         wf.setnchannels(CHANNELS)
         wf.setsampwidth(p.get_sample_size(FORMAT))
         wf.setframerate(RATE)
         wf.writeframes(b''.join(frames))
         wf.close()

     def recognizeyn():
         #model = whisper.load_model("small")
         word = "String"
         audio = whisper.load_audio("output.wav")
         audio = whisper.pad_or_trim(audio)
         dictionary = ["yes","no","イエス","ノー","Yes","No"]
         yesdic = ["Yes","yes","イエス"]
         nodic = ["No","no","ノー"]

         mel = whisper.log_mel_spectrogram(audio).to(model.device)

         _,probs = model.detect_language(mel)
         print(f"Detected language:{max(probs,key=probs.get)}")

         options = whisper.DecodingOptions(fp16 = False)
         result = whisper.decode(model, mel, options)
         words = result.text.split()
         matched_words = []
         print(result.text)
         '''
         for word in words:
             if word.lower() in dictionary:
                 matched_words.append(word)

         print("Match:",matched_words)
         '''
         #print(type(result.text))
         
         if "Yes" in result.text:
             text = "Yes"
         elif "yes" in result.text:
             text = "Yes"   
         elif "YES" in result.text:
             text = "Yes"
         elif "イエス" in result.text:
             text = "Yes"
         elif "No" in result.text:
             text = "No"
         elif "no" in result.text:
             text = "No"
         else:
             text = "False"
         
         return text
     
     def voice_yesno():
         text = String()
         YesNo.voiceyn()
         text = YesNo.recognizeyn()

         return text

#食品認識

class Food():
     def __init__(self):
         #model = whisper.load_model("small")
         print("Food")

     def voicefd():
         #model = whisper.load_model("small")
         CHUNK = 1024
         FORMAT = pyaudio.paInt16
         CHANNELS = 1
         RATE = 44100
         RECORD_SECONDS = 5
         WAVE_OUTPUT_FILENAME = "output.wav"

         p = pyaudio.PyAudio()

         stream = p.open(format = FORMAT,
                    channels = CHANNELS,
                    rate = RATE,
                    input = True,
                    #output = True,
                    #input_device_index = 2,
                    #output_device_index = 1,
                    frames_per_buffer = CHUNK)

         print("recording")
         tts.say("注文を取ります、お話ください")
         frames = []

         for i in range(0,int(RATE / CHUNK * RECORD_SECONDS)):
             d = stream.read(CHUNK)
             frames.append(d)

         print("done recording")

         stream.stop_stream()
         stream.close()
         p.terminate()

         wf = wave.open(WAVE_OUTPUT_FILENAME,'wb')
         wf.setnchannels(CHANNELS)
         wf.setsampwidth(p.get_sample_size(FORMAT))
         wf.setframerate(RATE)
         wf.writeframes(b''.join(frames))
         wf.close()

     def recognizefd():
         #model = whisper.load_model("small")
         foodlist = []
         how = 0
         #foodtext = String()
         word = "String"
         audio = whisper.load_audio("output.wav")
         audio = whisper.pad_or_trim(audio)
         dictionary = ["cola","Cola","green tea","Green Tea","cup","Cup"]


         mel = whisper.log_mel_spectrogram(audio).to(model.device)

         _,probs = model.detect_language(mel)
         print(f"Detected language:{max(probs,key=probs.get)}")

         options = whisper.DecodingOptions(fp16 = False)
         result = whisper.decode(model, mel, options)
         words = result.text.split()
         matched_words = []
         print(result.text)
         '''
         for word in words:
             if word.lower() in dictionary:
                 matched_words.append(word)

         print("Match:",matched_words)
         '''
         if "Cola" in result.text:
             #foodtext = "Cola"
             foodlist.append("Cola")
             how = how + 1
         elif "cola" in result.text:
             #foodtext = "Cola"
             foodlist.append("Cola")
             how = how + 1
         elif "コーラ" in result.text:
             foodlist.append("Cola")
             how = how + 1
         if "Tea" in result.text:
             #foodtext = "Green Tea"
             foodlist.append("Green Tea")
             how = how + 1
         elif "tea" in result.text:
             foodlist.append("Green Tea")
             how = how + 1
         elif "ティー" in result.text:
             foodlist.append("Green Tea")
             how = how + 1
         if how == 0:
             foodlist.append("False")

         return foodlist,how

     def voice_food():
         fod = "String"
         Food.voicefd()
         fod = Food.recognizefd()

         return fod

class Gpsr():
    def __init__(self):
        print("Gpsr")

    def VoiceGpsr(self):
        CHUNK = 1024
        FORMAT = pyaudio.paInt16
        CHANNELS = 1
        RATE = 44100
        RECORD_SECONDS = 10
        WAVE_OUTPUT_FILENAME = "output.wav"

        p = pyaudio.PyAudio()

        stream = p.open(format = FORMAT,
                        channels = CHANNELS,
                        rate = RATE,
                        input = True,
                        #output = True,
                        #input_device_index = 2,
                        #output_device_index = 1,
                        frames_per_buffer = CHUNK)

        print("recording")
        tts.say("お話ください")
        frames = []

        for i in range(0,int(RATE / CHUNK * RECORD_SECONDS)):
            d = stream.read(CHUNK)
            frames.append(d)

        print("done recording")

        stream.stop_stream()
        stream.close()
        p.terminate()

        wf = wave.open(WAVE_OUTPUT_FILENAME,'wb')
        wf.setnchannels(CHANNELS)
        wf.setsampwidth(p.get_sample_size(FORMAT))
        wf.setframerate(RATE)
        wf.writeframes(b''.join(frames))
        wf.close()

    def recognizeGpsr(self):
        #word = "String"
        Do_list = []
        Room1 = String()
        Room2 = String()
        how = 0
        audio = whisper.load_audio("output.wav")
        audio = whisper.pad_or_trim(audio)
        ver_list = ["Tell","grasp","find","follow","guide","answer","ask"]
        obj_list = ["water","milk tea","biscuits","corn soup","apple","lemon","bowl","mug","Amelia Angel","Angel Charlie","Charlie Jack","Charlotte Max","Hunter Noah","Mia Parker","Olivia Sam","Parker Thomas","Sam William"]
        word_list = ["room A","room B","shelf A","shelf B","living room","dynig room","drawer","long table A","bin A","bin B","tall table","long table B","chair A","cHair B","shelf"]
        objco_list = ["food"]
        pose_list = ["sit","stand","over"]

        mel = whisper.log_mel_spectrogram(audio).to(model.device)

        _,probs = model.detect_language(mel)

        options = whisper.DecodingOptions(fp16 = False)
        result = whisper.decode(model, mel, options)
        print(result.text)
        #text_list = result.text.split()

        if "Go" in result.text:
            how = how + 1
            Do_list.append("Go")
            for i in range(len(word_list)):
                if word_list[i] in result.text:
                    how = how + 1
                    Room1 = word_list[i]
                    Do_list.append(word_list[i])
                    break
            for n in range(len(ver_list)):
                if ver_list[n] in result.text:
                    how = how + 1
                    Do_list.append(ver_list[n])
                    break
            for m in range(len(obj_list)):
                if obj_list[m] in result.text:
                    how = how + 1
                    obj = obj_list[m]
                    Do_list.append(obj_list[m])
                    break
            for o in range(len(word_list)):
                if word_list[o] != Room1 and word_list[o] in result.text:
                    how = how + 1
                    Room2 = word_list[o]
                    Do_list.append(word_list[o])
                    break
            if "place" in result.text:
                how = how + 1
                Do_list.append("place")
                for p in range(len(word_list)):
                    if word_list[p] != Room1 and word_list[p] != Room2 and word_list[p] in result.text:
                        how = how + 1
                        Do_list.append(word_list[p])
                        break
            elif "give" in result.text:
                how = how + 1
                Do_list.append("give")
                for q in range(len(word_list)):
                    if obj_list[q] != obj and obj_list[q] in result.text:
                        how = how + 1
                        Do_list.append(obj_list[q])
                        break
            elif "answer" in result.text:
                if "his" in result.text:
                    how = how + 1
                    Do_list.append("his")
                else:
                    how = how + 1
                    Do_list.append("her")
            elif "ask" in result.text:
                if "him" in result.text:
                    how = how + 2
                    Do_list.append("him")
                    Do_list.append("Question")
                else:
                    how = how + 2
                    Do_list.append("her")
                    Do_list.append("Question")

        elif "Bring" in result.text:
            how = how + 1
            Do_list.append("Bring")
            for i in range(len(word_list)):
                if word_list[i] in result.text:
                    how = how + 1
                    Do_list.append(word_list[i])
                    break
            for n in range(len(ver_list)):
                if ver_list[n] in result.text:
                    how = how + 1
                    Do_list.append(ver_list[n])
                    break
            for m in range(len(obj_list)):
                if obj_list[m] in result.text:
                    how = how + 1
                    Do_list.append(obj_list[m])
                    break

        elif "Tell" in result.text:
            how = how + 1
            Do_list.append("Tell")
            for i in range(len(objco_list)):
                if objco_list[i] in result.text:
                    how = how + 1
                    Do_list.append(objco_list[i])
                    for m in range(len(word_list)):
                        if word_list[m] in result.text:
                            how = how + 1
                            Do_list.append(word_list[m])
                            break
            for n in range(len(word_list)):
                if word_list[n] in result.text:
                    how = how + 1
                    Do_list.append(word_list)
                    for o in range(len(pose_list)):
                        if pose_list[o] in result.text:
                            how = how + 1
                            Do_list.append(pose_list[o])
                            break
        else:
            how = 0
            Do_list.append("False")
        print(Do_list)

        return Do_list,result.text,how

    def Voice(self):
        Do_list = []
        Gpsr.VoiceGpsr(self)
        Do_list,text,how = Gpsr.recognizeGpsr(self)

        return text,Do_list,how



