package frc.robot.utils;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.HashMap;
import java.util.Map;

public class KeyboardController {
  private final int numButtons;

  private NetworkTableInstance inst;
  private NetworkTable keyboardTable;
  BooleanSubscriber[] buttonSubscribers;
  BooleanSubscriber isConnectedSubscriber;

  private final Map<EventLoop, Map<Integer, Trigger>> m_buttonCache = new HashMap<>();

  // Group of triggers for each subsystem
  private final AlgaeCollector algaeCollector;
  private final Climber climber;
  private final CoralCollector coralCollector;
  private final Elevator elevator;
  private final Wrist wrist;

  public KeyboardController(int port) {
    this(port, 80);
  }

  public KeyboardController(int port, int numButtons) {
    this.numButtons = numButtons;
    this.inst = NetworkTableInstance.getDefault();
    this.keyboardTable = inst.getTable("/AdvantageKit/DriverStation/Keyboard" + port);

    this.algaeCollector = new AlgaeCollector(this);
    this.climber = new Climber(this);
    this.coralCollector = new CoralCollector(this);
    this.elevator = new Elevator(this);
    this.wrist = new Wrist(this);

    buttonSubscribers = new BooleanSubscriber[this.numButtons];
    for (int i = 0; i < this.numButtons; i++) {
      buttonSubscribers[i] = keyboardTable.getBooleanTopic(String.valueOf(i)).subscribe(false);
    }
    isConnectedSubscriber = keyboardTable.getBooleanTopic("isConnected").subscribe(false);
  }

  /**
   * Constructs an event instance around this button's digital signal.
   *
   * @param button the button index
   * @return an event instance representing the button's digital signal attached to the {@link
   *     CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   * @see #button(int, EventLoop)
   */
  public Trigger button(int button) {
    return button(button, CommandScheduler.getInstance().getDefaultButtonLoop());
  }

  /**
   * Constructs an event instance around this button's digital signal.
   *
   * @param button the button index
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the button's digital signal attached to the given loop.
   */
  public Trigger button(int button, EventLoop loop) {
    var cache = m_buttonCache.computeIfAbsent(loop, k -> new HashMap<>());
    return cache.computeIfAbsent(
        button, k -> new Trigger(loop, () -> getRawButton(k) && isConnected()));
  }

  /**
   * Constructs an event instance around this button's digital signal.
   *
   * @param row the row index
   * @param col the column index
   * @return an event instance representing the button's digital signal attached to the {@link
   *     CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
   * @see #button(int, EventLoop)
   */
  public Trigger button(int row, int col) {
    return button(8 * (col - 1) + row - 1);
  }

  /**
   * Get if the HID is connected.
   *
   * @return true if the HID is connected
   */
  public boolean isConnected() {
    return isConnectedSubscriber.get();
  }

  private boolean getRawButton(int button) {
    return buttonSubscribers[button].get();
  }

  public AlgaeCollector algaeCollector() {
    return algaeCollector;
  }

  public Climber climber() {
    return climber;
  }

  public CoralCollector coralCollector() {
    return coralCollector;
  }

  public Elevator elevator() {
    return elevator;
  }

  public Wrist wrist() {
    return wrist;
  }

  public Trigger resetHeading() {
    return button(2, 10);
  }

  public static final record TestSubsystem(KeyboardController controller) {
    public Trigger test() {
        return controller.button(1, 1);
    }
  }

  public static final record AlgaeCollector(KeyboardController controller) {
    public Trigger collect() {
        return controller.button(6, 4)
            .or(controller.button(6, 5));
    }

    public Trigger release() {
        return controller.button(7, 4)
            .or(controller.button(7, 5));
    }

    public Trigger storage() {
        return controller.button(6,2)
            .or(controller.button(6, 3));
    }

    public Trigger reef() {
        return controller.button(7, 2)
            .or(controller.button(7, 3));
    }

    public Trigger ground() {
        return controller.button(8, 2)
            .or(controller.button(8, 3));
    }
  }

  public static final record Climber(KeyboardController controller) {
    public Trigger up() {
        return controller.button(2, 4)
            .or(controller.button(2, 5));
    }

    public Trigger down() {
        return controller.button(3, 4)
            .or(controller.button(3, 5));
    }
  }

  public static final record CoralCollector(KeyboardController controller) {
    public Trigger score() {
        return controller.button(2,2)
            .or(controller.button(2,3));
    }

    public Trigger reverse() {
        return controller.button(3,2)
            .or(controller.button(3,3));
    }
  }

  public static final record Elevator(KeyboardController controller) {
    public Trigger up() {
        return controller.button(2, 7)
            .or(controller.button(2, 8));
    }

    public Trigger down() {
        return controller.button(2, 9)
            .or(controller.button(2, 10));
    }

    public Trigger algaeBarge() {
        return controller.button(4, 7)
            .or(controller.button(4, 8));
    }

    public Trigger algaeL3() {
        return controller.button(5, 7)
            .or(controller.button(5, 8));
    }

    public Trigger algaeL2() {
        return controller.button(6, 7)
            .or(controller.button(6, 8));
    }

    public Trigger algaeProcessor() {
        return controller.button(7, 7)
            .or(controller.button(7, 8));
    }

    public Trigger coralL4() {
        return controller.button(4, 9)
            .or(controller.button(4, 10));
    }

    public Trigger coralL3() {
        return controller.button(5, 9)
            .or(controller.button(5, 10));
    }

    public Trigger coralL2() {
        return controller.button(6, 9)
            .or(controller.button(6, 10));
    }

    public Trigger coralL1() {
        return controller.button(7, 9)
            .or(controller.button(7, 10));
    }

    public Trigger storage() {
        return controller.button(8, 7)
            .or(controller.button(8, 8))
            .or(controller.button(8, 9))
            .or(controller.button(8, 10));
    }
  }

  public static final record Wrist(KeyboardController controller) {
    public Trigger rotateDown() {
        return controller.button(6, 1);
    }

    public Trigger rotateUp() {
        return controller.button(7, 1);
    }
  }
}
