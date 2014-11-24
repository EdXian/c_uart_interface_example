#include "send_offboard.h"

int serial_set_offboard(int serial_fd)
{
	int fd = serial_fd;

	mavlink_status_t lastStatus;
	lastStatus.packet_rx_drop_count = 0;

	unsigned loopcounter = 0;

	char buf[300];

	mavlink_message_t msg;

	mavlink_command_long_t com;

	com.param1 = 1.0f;				//A number > 0.5f is required here, according to MAV_CMD_NAV_GUIDED_ENABLE docs

	com.target_system = sysid;		//Not entirely sure what these are, but the example code uses these so I stuck with them
	com.target_component = compid_all;

	com.command = MAV_CMD_NAV_GUIDED_ENABLE;

	mavlink_msg_command_long_encode(sysid, compid_all, &msg, &com);

	unsigned len = mavlink_msg_to_send_buffer((uint8_t*)buf, &msg);

	printf("\nSending offboard command...\n");
	/* write packet via serial link */
	if(write(fd, buf, len) < 0)
	{
		printf("There was a writing error....\n");

		return -1;
	}

	/* wait until all data has been written */
	tcdrain(fd);
	printf("Set to offboard sent\n");
	/* Offboard config data sent */

	return 0;
}



int program_start(int argc, char **argv)
{

	/* default values for arguments */
	char *uart_name = (char*)"/dev/ttyUSB0";
	int baudrate = 57600;
	const char *commandline_usage = "\tusage: %s -d <devicename> -b <baudrate> [-v/--verbose] [--debug]\n\t\tdefault: -d %s -b %i\n";

	/* read program arguments */
	int i;

	for (i = 1; i < argc; i++)
	{
		/* argv[0] is "mavlink" */
		if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0)
		{
			printf(commandline_usage, argv[0], uart_name, baudrate);
			return 0;
		}

		/* UART device ID */
		if (strcmp(argv[i], "-d") == 0 || strcmp(argv[i], "--device") == 0)
		{
			if (argc > i + 1)
			{
				uart_name = argv[i + 1];

			}

			else
			{
				printf(commandline_usage, argv[0], uart_name, baudrate);
				return 0;
			}
		}

		/* baud rate */
		if (strcmp(argv[i], "-b") == 0 || strcmp(argv[i], "--baud") == 0)
		{
			if (argc > i + 1)
			{
				baudrate = atoi(argv[i + 1]);

			}

			else
			{
				printf(commandline_usage, argv[0], uart_name, baudrate);
				return 0;
			}
		}

		/* terminating MAVLink is allowed - yes/no */
		if (strcmp(argv[i], "-v") == 0 || strcmp(argv[i], "--verbose") == 0)
		{
			verbose = true;
		}

		if (strcmp(argv[i], "--debug") == 0)
		{
			debug = true;
		}
	}

	/* setup CTRL-C handling */
	signal(SIGINT, quit_handler);

	// SETUP SERIAL PORT

	// Exit if opening port failed
	// Open the serial port.
	if (!silent) printf("Trying to connect to %s.. ", uart_name);
	fflush(stdout);

	fd = open_port(uart_name);
	if (fd == -1)
	{
		if (!silent) printf("failure, could not open port.\n");
		exit(EXIT_FAILURE);
	}

	else
	{
		if (!silent) printf("success.\n");
	}

	if (!silent) printf("Trying to configure %s.. ", uart_name);

	bool setup = setup_port(fd, baudrate, 8, 1, false, false);

	if (!setup)
	{
		if (!silent) printf("failure, could not configure port.\n");
		exit(EXIT_FAILURE);
	}
	else
	{
		if (!silent) printf("success.\n");
	}

	int noErrors = 0;
	if (fd == -1 || fd == 0)
	{
		if (!silent) fprintf(stderr, "Connection attempt to port %s with %d baud, 8N1 failed, exiting.\n", uart_name, baudrate);
		exit(EXIT_FAILURE);
	}
	else
	{
		if (!silent) fprintf(stderr, "\nConnected to %s with %d baud, 8 data bits, no parity, 1 stop bit (8N1)\n", uart_name, baudrate);

		return 0;
	}

	if(fd < 0)
	{
		exit(noErrors);
	}
}

int serial_stream_coords(int serial_fd)
{
	int fd = serial_fd;

	mavlink_status_t lastStatus;
	lastStatus.packet_rx_drop_count = 0;

	unsigned loopcounter = 0;

	char buf[300];

	mavlink_message_t pos_msg;
	mavlink_set_position_target_local_ned_t pos;
	mavlink_local_position_ned_t cur_pos;

	/* Set coordinates */
	pos.x =cur_pos.x;
	pos.y = cur_pos.y;
	pos.z = -1.0f;		// Set to 1m above the ground (+z points DOWN)

	pos.coordinate_frame = MAV_FRAME_LOCAL_NED;

	pos.target_system = sysid;
	pos.target_component = compid_all;

	mavlink_msg_set_position_target_local_ned_encode(sysid, compid_all, &pos_msg, &pos);

	unsigned len = mavlink_msg_to_send_buffer((uint8_t*)buf, &pos_msg);

	printf("\nSending coordinate command...\n");
	/* write packet via serial link */
	if(write(fd, buf, len) < 0)
	{
		printf("There was a writing error....\n");

		return -1;
	}

	/* wait until all data has been written */
	tcdrain(fd);
	printf("Offboard coord sent\n");
	/* Offboard config data sent */

	/* Set attitude */
	mavlink_set_attitude_target_t att;
	mavlink_attitude_t cur_att;
	mavlink_message_t att_msg;

	att.thrust = 0.2f;
	att.body_yaw_rate = 0.1f;
	att.body_pitch_rate = cur_att.pitchspeed;
	att.body_roll_rate = cur_att.rollspeed;

	att.target_component = compid_all;
	att.target_system = sysid;

	mavlink_msg_set_attitude_target_encode(sysid, compid_all, &att_msg, &att);
	len = mavlink_msg_to_send_buffer((uint8_t*)buf, &att_msg);

	printf("\nSending attitude command...\n");
	/* write packet via serial link */
	if(write(fd, buf, len) < 0)
	{
		printf("There was a writing error....\n");

		return -1;
	}

	/* wait until all data has been written */
	tcdrain(fd);
	printf("Offboard attitude command sent\n");
	/* Offboard config data sent */

	return 0;
}

int main(int argc, char **argv)
{
	if(!program_start(argc, argv))
	{
		// Run indefinitely while the serial loop handles data
		if (!silent) printf("\nREADY, sending data.\n");

		while(1)
		{
			static int set_offboard_c = 0;

			/* start streaming the coords before setting to offboard */
			serial_stream_coords(fd);

			//Set to offboard mode ONCE
			if(set_offboard_c++ == 0) serial_set_offboard(fd);

			usleep(250000); 	//Ensure frequency < 0.5s
		}
		close_port(fd);

		return 0;
	}

	else
	{
		printf("Failed to start.\n");
		return -1;
	}
}
