/**
 * @file light_control_broadcast.c
 * @brief This file contains the main application code for the nRF broadcast project.
 *
 * The code initializes the necessary modules, handles button events, and defines callbacks for the DECT PHY operations.
 * It also includes global variables and functions for transmitting and receiving data.
 * The main function sets up the application and starts the event loop. The devices function as broadcast devices rn.
 *
 *
 * TODO:
 * - Implement yhe Type 2 header and test if it works
 * - Test different types of configurations for the DECT PHY
 * - Add the same reporting as in broadcast.c
 * - Add different functionality: report temperature, etc.
 *    - A functionality: Based on the botton pressed in the sink device the RDs will light one LED or another. when a broadcast is received then the RD is informed to stop sending broadcast. When the rd turns off the RD sends a broadcast to inform the sink that it is off
 * FIXED:
 * - This code has problems with the transmission of the data. It is not being received by the other device
 *    - The phy_ctrl_field_common structure should follow a specific order due to endianness
 * FIXME:
 * - The sending a unicast message doesn't work properly. Must related to the formationg of the control header
 */

#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <dk_buttons_and_leds.h>
#include <nrf_modem_dect_phy.h>
#include <modem/nrf_modem_lib.h>
#include <zephyr/drivers/hwinfo.h>

LOG_MODULE_REGISTER(app);

// Global variables that are defined here but must be defined in a different config file somehow
#define CONFIG_CARRIER 1677
#define CONFIG_NETWORK_ID 91
#define CONFIG_TX_POWER 11
#define CONFIG_MCS 1
#define CONFIG_RX_PERIOD_S 1
#define CONFIG_TX_TRANSMISSIONS 30

// Overall global variables used for the application
#define DATA_LEN_MAX 32
#define MAX_RD_DEVICES 3

static uint16_t device_id;
int crc_errors = 0;
int rssi_average = 0;
int n = 0; // Counter for RSSI calculations
bool button_pressed = false;
uint32_t button_number = 0;

// Array to store device IDs of RDs found via the receive function
uint32_t rd_device_ids[MAX_RD_DEVICES];

// Exit the application
static bool EXIT = false;

BUILD_ASSERT(CONFIG_CARRIER, "Carrier must be configured according to local regulations");

/* Semaphore to synchronize modem calls. */
// Used to signal the completion of asynchronous operations initiated by the modem initiation process
K_SEM_DEFINE(opt_sem, 0, 1);

/* TYPE 1 (SHORT BROADCAST) FORMAT 000 header
 * The order has to be different than in the specification due to endianness
 *
 */
struct phy_ctrl_field_common_type_1
{

	uint32_t packet_length : 4;		 // The length of the packet transmission in subslots or slots. (1 subslot= 5 OFDM symbols)
	uint32_t packet_length_type : 1; // Indicates wether the transmission length is in indicated in subslots or slots
	uint32_t header_format : 3;		 // Defines the format of the control header
	uint32_t short_network_id : 8;	 // Short network ID of the RD
	// Short Radio Device ID or Transmitter Identity: For locally in the radio neighbourhood
	uint32_t transmitter_id_hi : 8;
	uint32_t transmitter_id_lo : 8;
	uint32_t df_mcs : 3;   // Data field modulation and coding scheme
	uint32_t reserved : 1; // Set to 0, ignored by receiver
	uint32_t transmit_power : 4;
	uint32_t pad : 24;
};

/* TYPE 2 FORMAT 000 header
 *
 */

struct phy_ctrl_field_common_type_2_000
{
	uint32_t packet_length : 4;		 // The length of the packet transmission in subslots or slots. (1 subslot= 5 OFDM symbols)
	uint32_t packet_length_type : 1; // Indicates wether the transmission length is in indicated in subslots or slots
	uint32_t header_format : 3;		 // Defines the format of the control header
	uint32_t short_network_id : 8;	 // Short network ID of the RD
	// Short Radio Device ID or Transmitter Identity: For locally in the radio neighbourhood
	uint32_t transmitter_id_hi : 8;
	uint32_t transmitter_id_lo : 8;
	uint32_t df_mcs : 4; // Data field modulation and coding scheme
	uint32_t transmit_power : 4;
	uint32_t receiver_id_hi : 8; // Short RD ID of the receiving RD
	uint32_t receiver_id_lo : 8;
	uint32_t HARQ_process : 3;	   // HARQ process number of the transmission
	uint32_t DF_ind : 1;		   // The transmitter toggles this bit to control whether the receiver combines this transmission with the previous content of the HARQ process
	uint32_t DF_red_ver : 2;	   // Defines the redudancy version number of the transmission
	uint32_t spatial_streams : 2;  // Number of spatial streams of the data field
	uint32_t feedback_info_hi : 4; // Feedback information
	uint32_t feedback_format : 4;  // Defines the coding of the feedback
	uint32_t feedback_info_lo : 8;
};

/* TYPE 2 FORMAT 001 header
 *
 */

struct phy_ctrl_field_common_type_2_001
{
	uint32_t packet_length : 4;		 // The length of the packet transmission in subslots or slots. (1 subslot= 5 OFDM symbols)
	uint32_t packet_length_type : 1; // Indicates wether the transmission length is in indicated in subslots or slots
	uint32_t header_format : 3;		 // Defines the format of the control header
	uint32_t short_network_id : 8;	 // Short network ID of the RD
	// Short Radio Device ID or Transmitter Identity: For locally in the radio neighbourhood
	uint32_t transmitter_id_hi : 8;
	uint32_t transmitter_id_lo : 8;
	uint32_t df_mcs : 4; // Data field modulation and coding scheme
	uint32_t transmit_power : 4;
	uint32_t receiver_id_hi : 8; // Short RD ID of the receiving RD
	uint32_t receiver_id_lo : 8;
	uint32_t reserved : 6;		   // Ignored
	uint32_t spatial_streams : 2;  // Number of spatial streams of the data field
	uint32_t feedback_info_hi : 4; // Feedback information
	uint32_t feedback_format : 4;  // Defines the coding of the feedback
	uint32_t feedback_info_lo : 8;
};

// Structure of the data that is transmitted

struct rx_data_packet
{
	uint32_t transmitter_id;
};

struct tx_data_packet
{
	uint32_t button_number;
};

// ETSI TS 103 636-2  spec 8.3.3 RSSI is reported every 0.5dbm
// if successful reception, calculate the average
int32_t calcRSSI(int16_t recrssi, int is_success)
{
	float resp = -20 - ((-recrssi - 1) * 0.5);
	// avg_new=avg_old+(value-avg_old)/n
	if (is_success)
	{
		n++;
		float new_average = rssi_average + (resp - rssi_average) / n;
		rssi_average = new_average;
	}
	return (int32_t)resp;
}

/*DEFINE CALLBACKS OF THE DECT PHY*/

// Callback after init operation
static void init(const uint64_t *time, int16_t temp, enum nrf_modem_dect_phy_err err, const struct nrf_modem_dect_phy_modem_cfg *cfg)
{
	if (err == 0)
	{
		LOG_INF("DECT Modem Init done, temperature: %d", temp);
	}
	else
	{
		LOG_ERR("DECT Modem Init FAILED, error: %d", err);
		EXIT = true;
		return;
	}
	k_sem_give(&opt_sem);
}
void deinit(const uint64_t *time, enum nrf_modem_dect_phy_err err)
{
	if (err)
	{
		LOG_ERR("Deinit failed, err %d", err);
		return;
	}
	LOG_DBG("DEINIT response time %" PRIu64 " Status %d", *time, err);
	k_sem_give(&opt_sem);
}

// Operation complete notification
static void op_complete(const uint64_t *time, int16_t temperature, enum nrf_modem_dect_phy_err err, uint32_t handle)
{
	LOG_DBG("operation_complete_cb Status %d, Temp %d, Handle %d, time %" PRIu64 "", err, temperature, handle, *time);
	k_sem_give(&opt_sem);
}

// Callback after RX stop operation
static void rx_stop(const uint64_t *time, enum nrf_modem_dect_phy_err err, uint32_t handle)
{
	LOG_DBG("RX stop cb time %" PRIu64 " status %d, handle %d", *time, err, handle);
	k_sem_give(&opt_sem);
}

// Physical Control Channel reception notification
// THis is only when the control data is received
static void pcc(const uint64_t *time,
				const struct nrf_modem_dect_phy_rx_pcc_status *status,
				const union nrf_modem_dect_phy_hdr *hdr)
{

	// TODO: Check how to the type from the header and then cast it to the correct type
	struct phy_ctrl_field_common_type_1 *header_1 = (struct phy_ctrl_field_common_type_1 *)hdr->type_1;

	LOG_INF("Received header from device ID %d, phy_header_valid %d, rssi_2 %d", header_1->transmitter_id_hi << 8 | header_1->transmitter_id_lo, status->header_status, status->rssi_2);

	// Store the transmitter ID in the device ID array if it is not already present
	uint16_t transmitter_id = header_1->transmitter_id_hi << 8 | header_1->transmitter_id_lo;
	bool found = false;
	for (int i = 0; i < MAX_RD_DEVICES; i++)
	{
		if (rd_device_ids[i] == transmitter_id)
		{
			found = true;
			break;
		}
	}
	if (!found)
	{
		for (int i = 0; i < MAX_RD_DEVICES; i++)
		{
			if (rd_device_ids[i] == 0)
			{
				rd_device_ids[i] = transmitter_id;
				break;
			}
		}
	}
}

// Physical Control Channel CRC error notification
static void pcc_crc_err(const uint64_t *time, const struct nrf_modem_dect_phy_rx_pcc_crc_failure *crc_failure)
{
	crc_errors++;
	int16_t resp = calcRSSI(crc_failure->rssi_2, 0);
	LOG_INF("PDC CRC ERROR, rssi_2, %d, crc error count, %d, continuing", resp, crc_errors);
}

void led_control(uint8_t button_number)
{
	dk_set_led_off(DK_LED1);
	dk_set_led_off(DK_LED2);
	dk_set_led_off(DK_LED3);
	dk_set_led_off(DK_LED4);

	switch (button_number)
	{
	case 1 /* constant-expression */:
		/* code */
		dk_set_led_on(DK_LED1);
		break;
	case 2:
		dk_set_led_on(DK_LED2);
		break;
	case 4:
		dk_set_led_on(DK_LED3);
		break;
	case 8:
		dk_set_led_on(DK_LED4);
		break;

	default:
		break;
	}
}
/* Physical Data Channel reception notification. */
static void pdc(const uint64_t *time,
				const struct nrf_modem_dect_phy_rx_pdc_status *status,
				const void *data, uint32_t len)
{
	struct rx_data_packet *packet = (struct rx_data_packet *)data;
	/* Received RSSI value is in fixed precision format Q14.1 */
	LOG_INF("RX(RSSI: %d.%d): %d",
			(status->rssi_2 / 2), (status->rssi_2 & 0b1) * 5, packet->transmitter_id);
}

static void pdc_crc_err(
	const uint64_t *time, const struct nrf_modem_dect_phy_rx_pdc_crc_failure *crc_failure)
{
	crc_errors++;
	int16_t resp = calcRSSI(crc_failure->rssi_2, 0);
	LOG_INF("PDC CRC ERROR, rssi_2, %d, crc error count, %d, continuing", resp, crc_errors);
}

/* RSSI measurement result notification. */
static void rssi(const uint64_t *time, const struct nrf_modem_dect_phy_rssi_meas *status)
{
	LOG_DBG("rssi cb time %" PRIu64 " carrier %d", *time, status->carrier);
}

/* Callback after link configuration operation. */
static void link_config(const uint64_t *time, enum nrf_modem_dect_phy_err err)
{
	LOG_DBG("link_config cb time %" PRIu64 " status %d", *time, err);
}

/* Callback after time query operation. */
static void time_get(const uint64_t *time, enum nrf_modem_dect_phy_err err)
{
	LOG_DBG("time_get cb time %" PRIu64 " status %d", *time, err);
}

/* Callback after capability get operation. */
static void capability_get(const uint64_t *time, enum nrf_modem_dect_phy_err err,
						   const struct nrf_modem_dect_phy_capability *capability)
{
	LOG_DBG("capability_get cb time %" PRIu64 " status %d", *time, err);
	LOG_INF("DECT NR+ version: %d", capability->dect_version);
	LOG_INF("Variant count: %d", capability->variant_count);
	for (int i = 0; i < capability->variant_count; i++)
	{
		LOG_INF("Variant %d:", i);
		LOG_INF("Power class: %d", capability->variant[i].power_class);
		LOG_INF("RX spatial streams: %d", capability->variant[i].rx_spatial_streams);
		LOG_INF("RX TX diversity: %d", capability->variant[i].rx_tx_diversity);
		LOG_INF("RX gain: %d", capability->variant[i].rx_gain);
		LOG_INF("MCS max: %d", capability->variant[i].mcs_max);
		LOG_INF("HARQ soft buffer size: %u", capability->variant[i].harq_soft_buf_size);
		LOG_INF("HARQ process count max: %d", capability->variant[i].harq_process_count_max);
		LOG_INF("HARQ feedback delay: %d", capability->variant[i].harq_feedback_delay);
		LOG_INF("Subcarrier scaling factor: %d", capability->variant[i].mu);
		LOG_INF("Fourier transform scaling factor: %d", capability->variant[i].beta);
	}
}

struct nrf_modem_dect_phy_callbacks dect_phy_callbacks = {
	.init = init,
	.deinit = deinit,
	.op_complete = op_complete,
	.rx_stop = rx_stop,
	.pcc = pcc,
	.pcc_crc_err = pcc_crc_err,
	.pdc = pdc,
	.pdc_crc_err = pdc_crc_err,
	.rssi = rssi,
	.link_config = link_config,
	.time_get = time_get,
	.capability_get = capability_get,
};

struct nrf_modem_dect_phy_init_params dect_phy_init_params = {
	.harq_rx_expiry_time_us = 5000000,
	.harq_rx_process_count = 4,
};

/**
 * @brief Initializes the modem.
 *
 * This function initializes the modem library and sets the DECT PHY callbacks.
 * It also initializes the DECT PHY with the specified parameters.
 *
 * The function does not return a value.
 */
int initialize_modem()
{
	int err;
	// Initialize the modem library
	err = nrf_modem_lib_init();
	if (err)
	{
		LOG_ERR("Failed to initialize modem library, error: %d", err);
		return err;
	}

	err = nrf_modem_dect_phy_callback_set(&dect_phy_callbacks);
	if (err)
	{
		LOG_ERR("nrf_modem_dect_phy_callback_set failed, err %d", err);
		return err;
	}

	err = nrf_modem_dect_phy_init(&dect_phy_init_params);
	if (err)
	{
		LOG_ERR("nrf_modem_dect_phy_init failed, err %d", err);
		return err;
	}
	return 0;
}

/**
 * @brief Transmits data using the DECT PHY.
 *
 * This function transmits data using the DECT PHY. It takes three parameters:
 * - `handle`: The handle to identify the operation at the application processor side callbacks.
 * - `data`: The data to be transmitted.
 * - `data_len`: The length of the data to be transmitted.
 *
 * The function returns 0 on success, or an error code on failure.
 */
static int transmit_broadcast(uint32_t handle, void *data, size_t data_len)
{
	int err;

	struct phy_ctrl_field_common_type_1 header =
		{
			.header_format = 0x0,
			.packet_length_type = 0x0, // Length in subslots
			.packet_length = 0x01,	   // 1 subslot- TODO: Find out how long this is (5 OFDM symbols)
			.short_network_id = (CONFIG_NETWORK_ID & 0xff),
			.transmitter_id_hi = (device_id >> 8),
			.transmitter_id_lo = (device_id & 0xff),
			.transmit_power = CONFIG_TX_POWER,
			.reserved = 0,
			.df_mcs = CONFIG_MCS,
		};

	struct nrf_modem_dect_phy_tx_params tx_op_params =
		{
			.start_time = 0,  // Transmit immediately
			.handle = handle, // UNIQUE-identify the operation at application processor side callbacks.
			.network_id = CONFIG_NETWORK_ID,
			.phy_type = 0,				 // PHY type 1
			.lbt_rssi_threshold_max = 0, // No LBT
			.carrier = CONFIG_CARRIER,
			.lbt_period = NRF_MODEM_DECT_LBT_PERIOD_MAX,
			.phy_header = (union nrf_modem_dect_phy_hdr *)&header, // Formats the header as a union of type 1 or type 2 header
			.data = data,										   // Data to be transmitted- Does not need to be formatted
			.data_size = data_len,

		};
	err = nrf_modem_dect_phy_tx(&tx_op_params);
	if (err != 0)
	{
		return err;
	}

	return 0;
}

/**
 * @brief Transmits data using HARQ (Hybrid Automatic Repeat Request) protocol.
 *
 * This function transmits data using the HARQ protocol for unicast transmission. It prepares the necessary header and parameters
 * for the transmission and calls the `nrf_modem_dect_phy_tx_harq` function to initiate the transmission.
 *
 * @param handle The handle to uniquely identify the operation at the application processor side callbacks.
 * @param data Pointer to the data to be transmitted. The data does not need to be formatted.
 * @param data_len The size of the data to be transmitted.
 * @param receiver_id The ID of the receiver device.
 * @return 0 if the transmission is successful, otherwise an error code.
 */
static int transmit_unicast(uint32_t handle, void *data, size_t data_len, uint32_t receiver_id)
{
	int err;

	// struct phy_ctrl_field_common_type_2_000 header =
	// 	{
	// 		.header_format = 0x0,
	// 		.packet_length_type = 0x0, // Length in subslots
	// 		.packet_length = 0x01,	   // 1 subslot- TODO: Find out how long this is (5 OFDM symbols)
	// 		.short_network_id = (CONFIG_NETWORK_ID & 0xff),
	// 		.transmitter_id_hi = (device_id >> 8),
	// 		.transmitter_id_lo = (device_id & 0xff),
	// 		.transmit_power = CONFIG_TX_POWER,
	// 		.df_mcs = CONFIG_MCS,
	// 		.receiver_id_hi = (receiver_id >> 8),
	// 		.receiver_id_lo = (receiver_id & 0xff),
	// 		.HARQ_process = 0x0,
	// 		.DF_ind = 0,
	// 		.DF_red_ver = 0,
	// 		.spatial_streams = 0x0, // Single spatial stream (0x11-Eight spatial streams)
	// 		.feedback_info_hi = 0x0,
	// 		.feedback_format = 0x0, // No feedback, Receiver shall ignore feedback info bits
	// 		.feedback_info_lo = 0x0,
	// 	};

	struct phy_ctrl_field_common_type_2_001 header =
		{
			.header_format = 0x001,
			.packet_length_type = 0x0, // Length in subslots
			.packet_length = 0x01,	   // 1 subslot- TODO: Find out how long this is (5 OFDM symbols)
			.short_network_id = (CONFIG_NETWORK_ID & 0xff),
			.transmitter_id_hi = (device_id >> 8),
			.transmitter_id_lo = (device_id & 0xff),
			.transmit_power = CONFIG_TX_POWER,
			.df_mcs = CONFIG_MCS,
			.receiver_id_hi = (receiver_id >> 8),
			.receiver_id_lo = (receiver_id & 0xff),
			.reserved = 0,
			.spatial_streams = 0x0, // Single spatial stream (0x11-Eight spatial streams)
			.feedback_info_hi = 0x0,
			.feedback_format = 0x0, // No feedback, Receiver shall ignore feedback info bits
			.feedback_info_lo = 0x0,
		};

	struct nrf_modem_dect_phy_tx_params tx_op_params =
		{
			.start_time = 0,  // Transmit immediately
			.handle = handle, // UNIQUE-identify the operation at application processor side callbacks.
			.network_id = CONFIG_NETWORK_ID,
			.phy_type = 1,				 // PHY type 2
			.lbt_rssi_threshold_max = 0, // No LBT
			.carrier = CONFIG_CARRIER,
			.lbt_period = NRF_MODEM_DECT_LBT_PERIOD_MAX,
			.phy_header = (union nrf_modem_dect_phy_hdr *)&header, // Formats the header as a union of type 1 or type 2 header
			.data = data,										   // Data to be transmitted- Does not need to be formatted
			.data_size = data_len,

		};
	err = nrf_modem_dect_phy_tx(&tx_op_params);
	if (err != 0)
	{
		return err;
	}

	return 0;
}

static int receive(uint32_t handle)
{
	int err;

	struct nrf_modem_dect_phy_rx_params rx_op_params =
		{
			.start_time = 0, // Start immediately
			.handle = handle,
			.network_id = CONFIG_NETWORK_ID,
			.mode = NRF_MODEM_DECT_PHY_RX_MODE_CONTINUOUS,		   // Continued automatic reception
			.rssi_interval = NRF_MODEM_DECT_PHY_RSSI_INTERVAL_OFF, // No RSSI reporting- TODO: Change this with another value for testing
			.link_id = NRF_MODEM_DECT_PHY_LINK_UNSPECIFIED,		   // Receive form any RD
			.rssi_level = -60,									   // RSSI level threshold
			.carrier = CONFIG_CARRIER,
			.duration = CONFIG_RX_PERIOD_S * MSEC_PER_SEC * NRF_MODEM_DECT_MODEM_TIME_TICK_RATE_KHZ,
			.filter.short_network_id = CONFIG_NETWORK_ID & 0xff,
			.filter.is_short_network_id_used = 1,
			/* listen for everything (broadcast mode used) */
			.filter.receiver_identity = 0,

		};

	err = nrf_modem_dect_phy_rx(&rx_op_params);
	if (err)
	{
		return err;
	}

	return 0;
}

int shut_down()
{
	int err;
	LOG_INF("Shutting down");

	err = nrf_modem_dect_phy_deinit();
	if (err)
	{
		LOG_ERR("nrf_modem_dect_phy_deinit() failed, err %d", err);
		return err;
	}

	k_sem_take(&opt_sem, K_FOREVER);

	err = nrf_modem_lib_shutdown();
	if (err)
	{
		LOG_ERR("nrf_modem_lib_shutdown() failed, err %d", err);
		return err;
	}

	LOG_INF("Aiooo!");
	return 0;
}

/**
 * @brief Handles button events.
 *
 * This function is called when a button state changes. It takes two parameters:
 * - `button_state`: The current state of the button(s).
 * - `has_changed`: A bitmask indicating which button(s) have changed state.
 *
 * The function does not return a value.
 *
 * This function shouldn't transmit the value, the transmit should be correctly transmitted in the main loop so it's correctly handled via the semaphores
 */
void button_handler(uint32_t button_state, uint32_t has_changed)
{

	// Format the messsage before is sent
	button_pressed = true;
	if (button_state != 0)
	{
		printk("Button state: %d\n", button_state);
		button_number = button_state;
	}
}

int main(void)
{
	int err;
	size_t tx_len;
	uint8_t tx_buf[DATA_LEN_MAX];
	uint32_t receiver_id = 0;

	printk("Started new app\n");

	// Initialize buttons and LEDs
	dk_buttons_init(button_handler);
	printk("Buttons init\t");
	dk_leds_init();
	printk("LEDs init\n");

	initialize_modem();

	// Program waits for the initialitation of the modem to be completed
	k_sem_take(&opt_sem, K_FOREVER);
	if (EXIT)
	{
		return -EIO;
	}

	hwinfo_get_device_id((void *)&device_id, sizeof(device_id));

	LOG_INF("Dect NR+ PHY initialized, device ID: %d", device_id);

	// Get the capabilities of the DECT PHY defined in the modem
	err = nrf_modem_dect_phy_capability_get();
	if (err)
	{
		LOG_ERR("nrf_modem_dect_phy_capability_get failed, err %d", err);
	}

	// MAIN PROGRAM LOOP
	while (1)
	{
		// Listen for messages
		err = receive(200);

		if (err)
		{
			LOG_ERR("Receive failed, err %d", err);
			return err;
		}

		// /* Wait for RX operation to complete. */
		k_sem_take(&opt_sem, K_FOREVER);

		if (button_pressed == true && rd_device_ids[0] != 0)
		{
			switch (button_number)
			{
			case 1:
				receiver_id = rd_device_ids[0];
				break;
			case 2:
				receiver_id = rd_device_ids[1];
				break;
			case 4:
				receiver_id = rd_device_ids[2];
				break;
			case 8:
				break;
			default:
				break;
			}
			struct tx_data_packet data =
				{
					.button_number = button_number,
				};
			// TODO: The error control should be implemented in the transmit function and it shouldn't shout down when an error occurs
			err = transmit_unicast(0, &data, sizeof(data), receiver_id);
			if (err)
			{
				LOG_ERR("Transmit failed, err %d", err);
			}

			// /* Wait for TX operation to complete. */
			k_sem_take(&opt_sem, K_FOREVER);
			LOG_INF("TX:%d", data.button_number);
			button_pressed = false;
		}
	}

	return shut_down();
}