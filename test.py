import speech_recognition as sr

def test_microphone():
    """Simple test to see if voice recognition works"""
    
    recognizer = sr.Recognizer()
    
    print("=== Testing Microphone ===\n")
    
    # List all microphones
    print("Available microphones:")
    for index, name in enumerate(sr.Microphone.list_microphone_names()):
        print(f"  {index}: {name}")
    
    print("\n--- Starting voice test ---")
    print("Commands to try:")
    print("  English: forward, back, left, right, stop")
    print("  French: avancer, reculer, gauche, droite, arrêt")
    print("\nPress Ctrl+C to stop\n")
    
    try:
        with sr.Microphone() as source:
            print("🎤 Adjusting for background noise... (wait 2 seconds)")
            recognizer.adjust_for_ambient_noise(source, duration=2)
            print(f"✓ Ready! Energy threshold: {recognizer.energy_threshold}\n")
            
            test_count = 0
            while test_count < 5:  # Test 5 times then stop
                print(f"\n[Test {test_count + 1}/5] 🎤 Speak now...")
                
                try:
                    # Listen for audio
                    audio = recognizer.listen(source, timeout=5, phrase_time_limit=5)
                    print("⏳ Processing...")
                    
                    # Try French first
                    try:
                        text_fr = recognizer.recognize_google(audio, language='fr-FR')
                        print(f"✓ Heard (French): '{text_fr}'")
                        check_command(text_fr)
                        test_count += 1
                        continue
                    except:
                        pass
                    
                    # Try English
                    try:
                        text_en = recognizer.recognize_google(audio, language='en-US')
                        print(f"✓ Heard (English): '{text_en}'")
                        check_command(text_en)
                        test_count += 1
                        continue
                    except:
                        pass
                    
                    print("❌ Could not understand - please speak clearly")
                    
                except sr.WaitTimeoutError:
                    print("⏱️ No speech detected - try again")
                except sr.RequestError as e:
                    print(f"❌ Error: {e}")
                    print("⚠️ Check your internet connection!")
                    break
        
        print("\n✓ Test completed successfully!")
        
    except KeyboardInterrupt:
        print("\n\n⚠️ Test stopped by user")
    except Exception as e:
        print(f"\n❌ Error: {e}")

def check_command(text):
    """Check what command was detected"""
    text = text.lower()
    
    if any(word in text for word in ["avancer", "avance", "avant", "forward", "straight", "go"]):
        print("  → Command: FORWARD ⬆️")
    elif any(word in text for word in ["reculer", "recule", "arrière", "back", "backward", "reverse"]):
        print("  → Command: BACKWARD ⬇️")
    elif any(word in text for word in ["gauche", "left"]):
        print("  → Command: LEFT ⬅️")
    elif any(word in text for word in ["droite", "right"]):
        print("  → Command: RIGHT ➡️")
    elif any(word in text for word in ["stop", "arrêt", "arrête", "halt"]):
        print("  → Command: STOP 🛑")
    else:
        print("  → Command not recognized ❓")

if __name__ == "__main__":
    test_microphone()
