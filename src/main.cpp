#include <Arduino.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ESP8266WiFi.h>
#include <time.h>
#include <TZ.h>
#include <multi_channel_relay.h>
#include <ArduinoOTA.h>

#include "./Debouncer.h"
#include "./Credentials.h"

// ESP8266 PINS
// All pins are pulled down with resistors
#define GATE_BUTTON_PIN 16			// D0
#define LIGHT_BUTTON_PIN 14			// D5
#define GATE_DOWN_SWITCH_PIN 12 // D6
#define GATE_UP_SWITCH_PIN 13		// D7

#define TIMER_DECREASE_INTERVAL 10

// multi channel relay connections
#define LIGHT_CHANNEL 3
#define GATE_CHANNEL 2
#define STOP_CHANNEL 1
#define PHOTO_BAYPASS_CHANNEL 0

// debuging functions
#define DEBUG 0
#if DEBUG == 1
#define debug(x) Serial.print(x)
#define debugln(x) Serial.println(x)
#else
#define debug(x)
#define debugln(x)
#endif

// Let's Encrypt
// https://letsencrypt.org/certificates/
// ISRG Root X1 - pem
// June 4, 2035
const char *root_ca =
		"-----BEGIN CERTIFICATE-----\n"
		"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\n"
		"-----END CERTIFICATE-----\n";

BearSSL::WiFiClientSecure net;
PubSubClient mqttClient(net);

time_t now;

unsigned long lastObtainTimeTry = 0;
unsigned long lastTimerDecrease = 0;

#define MAX_CHANNELS 4

unsigned int channelTimer[MAX_CHANNELS] = {0};
bool changeChannelToOn[MAX_CHANNELS] = {false};
bool changeChannelToOff[MAX_CHANNELS] = {false};

bool isLightOn = false;
bool isPhotoBaypassOn = false;

enum class ConnectionState : unsigned int
{
	WIFI_CONNECTING,
	TIME_SYNCING,
	MQTT_CONNECTING,
	ONLINE
};

enum class GateState : unsigned int
{
	UNKNOWN,
	ERROR,
	OPEN,
	CLOSE,
	STOP,
	OPENING,
	CLOSING
};

ConnectionState connectionState = ConnectionState::WIFI_CONNECTING;
GateState gateState = GateState::UNKNOWN;

Multi_Channel_Relay relay;

Debouncer gateButton, lightButton, gateDownSwitch, gateUpSwitch;

void sendLightState()
{
	if (connectionState != ConnectionState::ONLINE)
	{
		return;
	}
	if (isLightOn)
	{
		mqttClient.publish(MQTT_CLIENT_ID "/light/state", "on", true);
	}
	else
	{
		mqttClient.publish(MQTT_CLIENT_ID "/light/state", "off", true);
	}
}

void setLightRelay()
{
	if (isLightOn)
	{
		changeChannelToOn[LIGHT_CHANNEL] = true;
	}
	else
	{
		changeChannelToOff[LIGHT_CHANNEL] = true;
	}
}

void setLightState(boolean isOn)
{
	isLightOn = isOn;
	setLightRelay();
	sendLightState();
}

void sendGateState()
{
	if (connectionState != ConnectionState::ONLINE)
	{
		return;
	}
	switch (gateState)
	{
	case GateState::CLOSE:
		mqttClient.publish(MQTT_CLIENT_ID "/gate/state", "close", true);
		break;
	case GateState::CLOSING:
		mqttClient.publish(MQTT_CLIENT_ID "/gate/state", "closing", true);
		break;
	case GateState::ERROR:
		mqttClient.publish(MQTT_CLIENT_ID "/gate/state", "error", true);
		break;
	case GateState::OPEN:
		mqttClient.publish(MQTT_CLIENT_ID "/gate/state", "open", true);
		break;
	case GateState::OPENING:
		mqttClient.publish(MQTT_CLIENT_ID "/gate/state", "opening", true);
		break;
	case GateState::STOP:
		mqttClient.publish(MQTT_CLIENT_ID "/gate/state", "stop", true);
		break;
	case GateState::UNKNOWN:
		mqttClient.publish(MQTT_CLIENT_ID "/gate/state", "unknown", true);
		break;
	}
}

void setGateState(GateState state)
{
	gateState = state;
	sendGateState();
}

void handleGateButtonClick()
{
	debugln("gate button click");
#define PUSH_BUTTON_TIME 100
	changeChannelToOn[GATE_CHANNEL] = true;
	channelTimer[GATE_CHANNEL] = PUSH_BUTTON_TIME / TIMER_DECREASE_INTERVAL;

	if (!isPhotoBaypassOn)
	{
// baypass photo stop for 3000 ms
#define TIME_TO_TURN_OFF_PHOTO_STOP 3000
		changeChannelToOn[PHOTO_BAYPASS_CHANNEL] = true;
		channelTimer[PHOTO_BAYPASS_CHANNEL] = TIME_TO_TURN_OFF_PHOTO_STOP / TIMER_DECREASE_INTERVAL;
	}

	if (gateState == GateState::OPENING)
	{
		setGateState(GateState::STOP);
	}
	else if (gateState == GateState::STOP)
	{
		setGateState(GateState::CLOSING);
	}
	else if (gateState == GateState::CLOSING)
	{
		setGateState(GateState::OPENING);
	}
}

void setPhotoBaypassRelay()
{
	if (isPhotoBaypassOn)
	{
		changeChannelToOn[PHOTO_BAYPASS_CHANNEL] = true;
	}
	else
	{
		changeChannelToOff[PHOTO_BAYPASS_CHANNEL] = true;
	}
}

void handleLightButtonClick()
{
	debugln("Light button click");
	setLightState(!isLightOn);
}

void handleGateSwitchChange()
{
	uint8_t downSwitchState = gateDownSwitch.getState();
	uint8_t upSwitchState = gateUpSwitch.getState();

	if (downSwitchState == HIGH && upSwitchState == HIGH)
	{
		setGateState(GateState::ERROR);
	}
	else if (gateState == GateState::CLOSE && downSwitchState == LOW && upSwitchState == LOW)
	{
		setLightState(true);
		setGateState(GateState::OPENING);
	}
	else if (gateState == GateState::OPEN && downSwitchState == LOW && upSwitchState == LOW)
	{
		setLightState(false);
		sendLightState();
		setGateState(gateState = GateState::CLOSING);
	}
	else if (downSwitchState == LOW && upSwitchState == HIGH)
	{
		setLightState(true);
		setGateState(gateState = GateState::OPEN);
	}
	else if (downSwitchState == HIGH && upSwitchState == LOW)
	{
		setLightState(false);
		sendLightState();
		setGateState(GateState::CLOSE);
	}
}

void mqtt_callback(char *topic, byte *payload, unsigned int length)
{
	debug("Message arrived [");
	debug(topic);
	debug("] ");
#ifdef DEBUG
	for (unsigned int i = 0; i < length; i++)
	{
		debug((char)payload[i]);
	}
#endif
	debugln("");

	if (strcmp(topic, MQTT_CLIENT_ID "/gate/switch") == 0)
	{
		handleGateButtonClick();
	}

	if (strcmp(topic, MQTT_CLIENT_ID "/light") == 0)
	{
		if (strncmp((char *)payload, "true", length) == 0)
		{
			setLightState(true);
		}
		else
		{
			setLightState(false);
		}
	}

	if (strcmp(topic, MQTT_CLIENT_ID "/potobaypass") == 0)
	{
		if (strncmp((char *)payload, "true", length) == 0)
		{
			isPhotoBaypassOn = true;
		}
		else
		{
			isPhotoBaypassOn = false;
		}
		setPhotoBaypassRelay();
	}
	if (strcmp(topic, MQTT_CLIENT_ID "/restart") == 0)
	{
		ESP.restart();
	}
}

void prepare_topic(char *topic, const char *text)
{
	strcpy(topic, MQTT_CLIENT_ID);
	strcat(topic, text);
}

void setup()
{

#ifdef DEBUG
	Serial.begin(250000);
#endif
	debugln("START");

	WiFi.mode(WIFI_STA);
	WiFi.disconnect();
	WiFi.begin(SSID, WIFI_PASSWORD);
	WiFi.setAutoReconnect(true);
	WiFi.persistent(true);

	relay.begin(0x11);

	gateButton.setPinMode(GATE_BUTTON_PIN, INPUT);
	gateButton.setTrigger(HIGH);
	gateButton.setDebounceTime(50);
	gateButton.setCallback(handleGateButtonClick);

	lightButton.setPinMode(LIGHT_BUTTON_PIN, INPUT);
	lightButton.setTrigger(HIGH);
	lightButton.setDebounceTime(50);
	lightButton.setCallback(handleLightButtonClick);

	gateDownSwitch.setPinMode(GATE_DOWN_SWITCH_PIN, INPUT);
	gateDownSwitch.setTrigger(BOTH);
	gateDownSwitch.setDebounceTime(250);
	gateDownSwitch.setCallback(handleGateSwitchChange);

	gateUpSwitch.setPinMode(GATE_UP_SWITCH_PIN, INPUT);
	gateUpSwitch.setTrigger(BOTH);
	gateUpSwitch.setDebounceTime(250);
	gateUpSwitch.setCallback(handleGateSwitchChange);

	mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
	mqttClient.setCallback(mqtt_callback);
}

void decrementTimers()
{
	static unsigned long lastTimerDecrease = millis();
	unsigned long elapsedTime = millis() - lastTimerDecrease;

	if (lastTimerDecrease > millis())
	{
		elapsedTime = 0;
		lastTimerDecrease = 0;
	}

	if (elapsedTime >= TIMER_DECREASE_INTERVAL)
	{
		for (int i = 0; i < MAX_CHANNELS; i++)
		{
			if (channelTimer[i] > 0)
			{
				channelTimer[i]--;
				if (channelTimer[i] == 0)
				{
					changeChannelToOff[i] = true;
				}
			}
		}
		lastTimerDecrease = millis();
	}
}

void relaysStateControl()
{
	for (int i = 0; i < 4; i++)
	{
		if (changeChannelToOn[i])
		{
			changeChannelToOn[i] = false;
			debug("turn on channel ");
			debugln(i + 1);
			relay.turn_on_channel(i + 1);
		}
		if (changeChannelToOff[i])
		{
			changeChannelToOff[i] = false;
			debug("turn off channel ");
			debugln(i + 1);
			relay.turn_off_channel(i + 1);
		}
	}
}

void stateMachine()
{

	switch (connectionState)
	{

	case ConnectionState::WIFI_CONNECTING:

		if (WiFi.status() == WL_CONNECTED)
		{
			debugln("WiFi CONNECTED");
#ifdef DEBUG
			IPAddress myIP = WiFi.localIP();
			debugln(myIP);
#endif
			ArduinoOTA.begin();
			connectionState = ConnectionState::TIME_SYNCING;
			debugln("TIME SYNCING");
		}
		break;

	case ConnectionState::TIME_SYNCING:

		if (millis() + 30000 - lastObtainTimeTry > 30000)
		{
			configTime(TZ_Europe_Warsaw, "ntp1.tp.pl", "tempus1.gum.gov.pl", "ntp.nask.pl");
			lastObtainTimeTry = millis() + 30000;
		}
		if (millis() + 30000 < lastObtainTimeTry)
		{
			lastObtainTimeTry = millis() + 30000;
		}
		now = time(nullptr);
		if (now > 100000000)
		{
			debugln(ctime(&now));
			connectionState = ConnectionState::MQTT_CONNECTING;
		}
		break;

	case ConnectionState::MQTT_CONNECTING:
		if (!mqttClient.connected())
		{
			debugln("MQTT CONNECTING");

			BearSSL::X509List cert(root_ca);
			net.setTrustAnchors(&cert);

			if (mqttClient.connect(MQTT_CLIENT_ID, MQTT_LOGIN, MQTT_PASSWORD, MQTT_WILL_TOPIC, 1, true, MQTT_WILL_MESSAGE, true))
			{
				debugln("MQTT CONNECTED");
				connectionState = ConnectionState::ONLINE;
				mqttClient.publish(MQTT_CLIENT_ID "/status", "online", true);
				mqttClient.subscribe(MQTT_CLIENT_ID "/light");
				mqttClient.subscribe(MQTT_CLIENT_ID "/gate/switch");
				mqttClient.subscribe(MQTT_CLIENT_ID "/restart");
				sendGateState();
				sendLightState();
			}
			else
			{
				debug("failed, rc=");
				debugln(mqttClient.state());
			}
		}
		break;

	case ConnectionState::ONLINE:
		if (WiFi.status() != WL_CONNECTED)
		{
			debugln("WiFi DISCONNECTED");
			connectionState = ConnectionState::WIFI_CONNECTING;
		}
		if (!mqttClient.connected())
		{
			debugln("MQTT DISCONNECTED");
			connectionState = ConnectionState::MQTT_CONNECTING;
		}
		break;
	}
}

void loop()
{
	stateMachine();
	if (connectionState != ConnectionState::WIFI_CONNECTING)
	{
		ArduinoOTA.handle();
	}
	if (connectionState == ConnectionState::ONLINE)
	{
		mqttClient.loop();
	}
	gateButton.loop();
	lightButton.loop();
	gateDownSwitch.loop();
	gateUpSwitch.loop();
	decrementTimers();
	relaysStateControl();
}
