import speech_recognition
import sys

def greeting(name: str):
    """Greeting function"""
    return f"Привет, {name}!"

if __name__ == "__main__":
    user_name = 'босс'
    sr = speech_recognition.Recognizer()
    sr.pause_threshold = 0.5

    with speech_recognition.Microphone() as mic:
        sr.adjust_for_ambient_noise(source=mic, duration=0.5)
        audio = sr.listen(source=mic)
        try:
            query = sr.recognize_google(audio_data=audio, language='ru-RU').lower()
        except speech_recognition.UnknownValueError:
            print(f"Простите, {user_name}, но я Вас не понял.")
            sys.exit(1)
    if 'привет' in query:
        print(greeting(user_name))
    else:
        print(f"Вы сказали: {query.capitalize()}.")



