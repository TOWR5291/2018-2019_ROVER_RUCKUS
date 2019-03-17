package club.towr5291.functions;

import android.util.Log;

import org.firstinspires.ftc.robotcore.external.android.AndroidTextToSpeech;

public class TOWR5291TextToSpeech {
    private AndroidTextToSpeech androidTextToSpeech;
    private float SpeakingRate = 1.5f;
    private float SpeakingPitch = 1f;
    private boolean Speak = true;

    public TOWR5291TextToSpeech(){
        this.androidTextToSpeech = new AndroidTextToSpeech();
        this.androidTextToSpeech.initialize();
        this.androidTextToSpeech.setLanguageAndCountry("en", "US");
        this.androidTextToSpeech.setSpeechRate(this.SpeakingRate);
        this.androidTextToSpeech.setPitch(this.SpeakingPitch);
    }

    public TOWR5291TextToSpeech(boolean speak){
        this.Speak = speak;
        this.androidTextToSpeech = new AndroidTextToSpeech();
        this.androidTextToSpeech.initialize();
        this.androidTextToSpeech.setLanguageAndCountry("en", "US");
        this.androidTextToSpeech.setSpeechRate(this.SpeakingRate);
        this.androidTextToSpeech.setPitch(this.SpeakingPitch);
    }

    public TOWR5291TextToSpeech(String languageCode){
        this.androidTextToSpeech = new AndroidTextToSpeech();
        this.androidTextToSpeech.initialize();
        this.androidTextToSpeech.setLanguageAndCountry(languageCode, "US");
        this.androidTextToSpeech.setSpeechRate(this.SpeakingRate);
        this.androidTextToSpeech.setPitch(this.SpeakingPitch);
    }

    public TOWR5291TextToSpeech(float SpeechRate, float SpeechPitch){
        this.androidTextToSpeech = new AndroidTextToSpeech();
        this.androidTextToSpeech.initialize();
        this.androidTextToSpeech.setLanguageAndCountry("en", "US");

        this.SpeakingRate = SpeechRate;
        this.SpeakingPitch = SpeechPitch;

        this.androidTextToSpeech.setSpeechRate(this.SpeakingRate);
        this.androidTextToSpeech.setPitch(this.SpeakingPitch);
    }

    /**
     * Call this and it sets the tone, pitch, and it checks to make sure that it is not speaking
     * @param Text
     */
    public void Speak(String Text){
        if (!this.androidTextToSpeech.isSpeaking() && this.Speak) {
            this.androidTextToSpeech.speak(Text);
        } else {
            Log.e("Speaking", "ERROR CAN NOT SPEAK TEXT ALREADY SPEAKING");
        }
    }

    public void Speak(String Text, int debug){
        if (!this.androidTextToSpeech.isSpeaking() && debug >= 8 && this.Speak){
            this.androidTextToSpeech.speak(Text);
        } else if (this.androidTextToSpeech.isSpeaking()){
            Log.e("Speaking", "ERROR CAN NOT SPEAK TEXT ALREADY SPEAKING");
        } else {
            Log.e("Speaking", "ERROR Debug not above 8");
        }
    }

    public boolean isSpeaking(){
        return this.androidTextToSpeech.isSpeaking();
    }

    /**
     * 1.0 is the regular rate any lower the slower the text is spoken and higher the faster
     * @param Rate
     */
    public void setSpeakingRate(float Rate){
        this.androidTextToSpeech.setSpeechRate(Rate);
    }

    /**
     * 1.0 is the normal Pitch, Lower the number the low the voice and Higher the Number the Higher
     * the pitch will be.
     * @param Pitch
     */
    public void setSpeakingPitch(float Pitch){
        this.androidTextToSpeech.setPitch(Pitch);
    }

    public void setSpeak(boolean speak){
        this.Speak = speak;
    }
}