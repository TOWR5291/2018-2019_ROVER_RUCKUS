package club.towr5291.functions;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Timer;

public class GameTime {

    enum GameSection{
        AUTO ("AUTO"),
        TELEOP ("TELEOP"),
        ENDGAME ("ENDGAME"),
        ERROR ("ERROR"),
        TEST ("TEST"),
        BETWEEN ("BETWEEN");

        private final String name;

        GameSection (String name) {
            this.name = name;
        }

        public String toString() {
            return name;
        }

    }
    private ElapsedTime gameTime;
    private double StopTime = 0;

    public GameTime(){
        gameTime = new ElapsedTime();
        gameTime.startTime();
    }

    public GameSection getGameTime(){
        if(gameTime.seconds() >= 30 + StopTime){
            return GameSection.TELEOP;
        } else if (gameTime.seconds() <= 30 + StopTime){
            return GameSection.AUTO;
        } else if (gameTime.seconds() >= 150 + StopTime){
            return GameSection.ENDGAME;
        } else {
            return null;
        }
    }

    public void Stop(){
        StopTime = gameTime.seconds();
        gameTime.reset();
    }

    public void Start(){
        gameTime.reset();
        gameTime.startTime();
    }
}
