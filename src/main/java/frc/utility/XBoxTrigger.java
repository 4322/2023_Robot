package frc.utility;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
	 * This class is used to represent one of the two
	 * Triggers on an Xbox360 controller.
	 */
	public class XBoxTrigger extends Trigger
	{
    public enum HAND
	  {
		  LEFT, RIGHT
	  }
    /* Default Values */
    private static final double DEFAULT_TRIGGER_DEADZONE = 0.01; // Jiggle room for the triggers
	  private static final double DEFAULT_TRIGGER_SENSITIVITY = 0.6;  // If the trigger is beyond this limit, say it has been pressed
    private static final int LEFT_TRIGGER_AXIS_ID = 2;
    private static final int RIGHT_TRIGGER_AXIS_ID = 3;
		/* Instance Values */
		private final Joystick parent;
		private final HAND hand;

		private double deadZone;
		private double sensitivity;


		/**
		 * Constructor
		 *
		 * @param joystick
		 * @param hand
		 */
		XBoxTrigger(final Joystick joystick, final HAND hand)
		{

            /* Initialize */
			this.parent = joystick;
			this.hand = hand;
			this.deadZone = DEFAULT_TRIGGER_DEADZONE;
			this.sensitivity = DEFAULT_TRIGGER_SENSITIVITY;
		}

    	/**
	 * Creates a deadzone, but without clipping the lower values.
	 * turns this
	 * |--1--2--3--4--5--|
	 * into this
	 * ______|-1-2-3-4-5-|
	 *
	 * @param input
	 * @param deadZoneSize
	 * @return adjusted_input
	 */
	private static double createDeadZone(double input, double deadZoneSize)
	{
		final double negative;
		double deadZoneSizeClamp = deadZoneSize;
		double adjusted;

		if(deadZoneSizeClamp < 0 || deadZoneSizeClamp >= 1)
		{
			deadZoneSizeClamp = 0;  // Prevent any weird errors
		}

		negative = input < 0 ? -1 : 1;

		adjusted = Math.abs(input) - deadZoneSizeClamp;  // Subtract the deadzone from the magnitude
		adjusted = adjusted < 0 ? 0 : adjusted;          // if the new input is negative, make it zero
		adjusted = adjusted / (1 - deadZoneSizeClamp);   // Adjust the adjustment so it can max at 1

		return negative * adjusted;
	}

		/* Extended Methods */
		@Override
		public boolean getAsBoolean()
		{
			return getX() > sensitivity;
		}



        /* Get Methods */

		/**
		 * getHand
		 *
		 * @return Trigger hand
		 * <p>
		 * See which side of the controller this trigger is
		 */
		public HAND getHand()
		{
			return hand;
		}

		/**
		 * 0 = Not pressed
		 * 1 = Completely pressed
		 *
		 * @return How far its pressed
		 */
		public double getX()
		{
			final double rawInput;

			if(hand == HAND.LEFT)
			{
				rawInput = parent.getRawAxis(LEFT_TRIGGER_AXIS_ID);
			}
			else
			{
				rawInput = parent.getRawAxis(RIGHT_TRIGGER_AXIS_ID);
			}

			return createDeadZone(rawInput, deadZone);
		}

		public double getY()
		{
			return getX();    // Triggers have one dimensional movement. Use getX() instead
		}



        /* Set Methods */

		/**
		 * Set the deadzone of this trigger
		 *
		 * @param number
		 */
		public void setTriggerDeadZone(double number)
		{
			this.deadZone = number;
		}

		/**
		 * How far you need to press this trigger to activate a button press
		 *
		 * @param number
		 */
		public void setTriggerSensitivity(double number)
		{
			this.sensitivity = number;
		}
	}