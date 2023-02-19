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


		/* Extended Methods */
		@Override
		public boolean get()
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