package il.ac.bgu.cs.bp.leaderfollower;

import java.io.BufferedWriter;
import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;
import java.text.DecimalFormat;
import java.time.Instant;
import java.util.Arrays;
import java.util.logging.Level;
import java.util.logging.Logger;
import javax.swing.SwingUtilities;
import il.ac.bgu.cs.bp.bpjs.model.BProgram;
import il.ac.bgu.cs.bp.leaderfollower.PlayerCommands.GpsData;
import il.ac.bgu.cs.bp.leaderfollower.events.StateUpdate;

public class StateUpdater implements Runnable {
  private final int maxSteps = 310;
  private Double[] d2TAllTime = new Double[maxSteps];
  private Double[] disAllTime = new Double[maxSteps];
  private int stepCount = 0;
  private final DecimalFormat fmt = new DecimalFormat("#0.000");
  private final BProgram bp;
  private final PlayerCommands player;
  private final ControlPanel controlPanel;
  private final GpsData playerGoal;
  private Thread thread;

  public StateUpdater(BProgram bp, PlayerCommands player, PlayerData data,
      ControlPanel controlPanel) {
    this.bp = bp;
    this.player = player;
    this.controlPanel = controlPanel;
    this.playerGoal = data.goal;
  }

  @Deprecated
  @Override
  public void run() {
    try {
      while (true) {
        Thread.sleep(200);
        collectTelemetry();
      }
    } catch (InterruptedException ex) {
      Logger.getLogger(BPJsRobotControl.class.getName()).log(Level.SEVERE, null, ex);
    }
  }

  @Deprecated
  public void start() {
    thread = new Thread(this);
    thread.start();
  }

  public void collectTelemetry() {
    GpsData playerGpsData, ballGpsData;
    Double playerCompassDeg;
    try {
      playerGpsData = player.getPlayerGps();
      ballGpsData = player.getBallGps();
      playerCompassDeg = player.getPlayerCompass();
    } catch (Exception e) {
      Logger.getLogger(BPJsRobotControl.class.getName()).log(Level.SEVERE, "NO TELEMETRY RECIEVED");
      return;
    }

    StateUpdate t = new StateUpdate(playerGpsData, ballGpsData, playerGoal, playerCompassDeg);

    SwingUtilities.invokeLater(() -> {
      controlPanel.PlayerGpsZ_Text.setText(fmt.format(t.playerGps.z));
      controlPanel.PlayerGpsX_Text.setText(fmt.format(t.playerGps.x));
      controlPanel.BallGpsZ_Text.setText(fmt.format(t.ballGps.z));
      controlPanel.BallGpsX_Text.setText(fmt.format(t.ballGps.x));
      controlPanel.Distance2Ball_Text.setText(fmt.format(t.ball.distance));
      controlPanel.PlayerDegree_Text.setText(fmt.format(t.playerCompass));
      controlPanel.Deg2Ball_Text.setText(fmt.format(t.ball.degree));
      controlPanel.Deg2Goal_Text.setText(fmt.format(t.goal.degree));
    });
    // Logger.getLogger(BPJsRobotControl.class.getName()).log(Level.INFO, "Enqueuing telemetry");
    bp.enqueueExternalEvent(t);
    /* if (stepCount < maxSteps) {
      System.out.println(stepCount);
      d2TAllTime[stepCount] = t.ball.degree;
      disAllTime[stepCount] = t.ball.distance;
    } else if (stepCount == maxSteps) {
      System.out.println("D2tArray: " + Arrays.toString(d2TAllTime));
      System.out.println("DistArray: " + Arrays.toString(disAllTime));
      try {
        writeToFile("SimDataDeg.csv", "DegToTarget: ", d2TAllTime);
        writeToFile("SimDataDist.csv", "DistanceToTarget: ", disAllTime);
        player.close();
        System.exit(0);
      } catch (IOException ex) {
        Logger.getLogger(BPJsRobotControl.class.getName()).log(Level.SEVERE, null, ex);
      }
    }
    stepCount++; */
    controlPanel.TimeLabel.setText(Integer.toString(stepCount));
  }

  // file read
  private static void writeToFile(String fileName, String dataInfo, Double[] theArray)
      throws IOException {
    FileWriter fW = new FileWriter(fileName, true);
    BufferedWriter bW = new BufferedWriter(fW);
    try {
      Instant a = Instant.now();
      bW.append("[" + a.toString() + "], " + dataInfo + ","
          + Arrays.toString(theArray).substring(1, Arrays.toString(theArray).length() - 1) + "\n");
    } catch (FileNotFoundException e) {
      e.printStackTrace();
    } finally {
      bW.flush();
      bW.close();
    }
  }
}
