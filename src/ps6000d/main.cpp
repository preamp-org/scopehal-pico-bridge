/***********************************************************************************************************************
*                                                                                                                      *
* ps6000d                                                                                                              *
*                                                                                                                      *
* Copyright (c) 2012-2026 Andrew D. Zonenberg and contributors                                                         *
* All rights reserved.                                                                                                 *
*                                                                                                                      *
* Redistribution and use in source and binary forms, with or without modification, are permitted provided that the     *
* following conditions are met:                                                                                        *
*                                                                                                                      *
*    * Redistributions of source code must retain the above copyright notice, this list of conditions, and the         *
*      following disclaimer.                                                                                           *
*                                                                                                                      *
*    * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the       *
*      following disclaimer in the documentation and/or other materials provided with the distribution.                *
*                                                                                                                      *
*    * Neither the name of the author nor the names of any contributors may be used to endorse or promote products     *
*      derived from this software without specific prior written permission.                                           *
*                                                                                                                      *
* THIS SOFTWARE IS PROVIDED BY THE AUTHORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED   *
* TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL *
* THE AUTHORS BE HELD LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES        *
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR       *
* BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT *
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE       *
* POSSIBILITY OF SUCH DAMAGE.                                                                                          *
*                                                                                                                      *
***********************************************************************************************************************/

/**
	@file
	@author Andrew D. Zonenberg
	@brief Program entry point
 */

#include "ps6000d.h"
#include "PicoSCPIServer.h"
#include <signal.h>

PICO_STATUS (*picoGetUnitInfo) (int16_t, int8_t *, int16_t, int16_t *, PICO_INFO);
PICO_STATUS (*picoGetUnitInfo2) (int16_t, int8_t *, int16_t, PICO_INFO);
PICO_INFO Open2000();
PICO_INFO Open3000();
PICO_INFO Open4000();
PICO_INFO Open5000();
PICO_INFO Open6000();

using namespace std;

void help();

void help()
{
	fprintf(stderr,
			"ps6000d [general options] [logger options]\n"
			"\n"
			"  [general options]:\n"
			"    --help                        : this message...\n"
			"    --series <number>             : specifies the model series to look for (2000, 3000, 4000, 5000, 6000)\n"
			"    --scpi-port port              : specifies the SCPI control plane port (default 5025)\n"
			"    --waveform-port port          : specifies the binary waveform data port (default 5026)\n"
			"\n"
			"  [logger options]:\n"
			"    levels: ERROR, WARNING, NOTICE, VERBOSE, DEBUG\n"
			"    --quiet|-q                    : reduce logging level by one step\n"
			"    --verbose                     : set logging level to VERBOSE\n"
			"    --debug                       : set logging level to DEBUG\n"
			"    --trace <classname>|          : name of class with tracing messages. (Only relevant when logging level is DEBUG.)\n"
			"            <classname::function>\n"
			"    --logfile|-l <filename>       : output log messages to file\n"
			"    --logfile-lines|-L <filename> : output log messages to file, with line buffering\n"
			"    --stdout-only                 : writes errors/warnings to stdout instead of stderr\n"
		   );
}

string g_model;
string g_serial;
string g_fwver;
size_t g_series;

PicoScopeType g_pico_type;
int16_t g_hScope = 0;
size_t g_numChannels = 0;
bool limitChannels = false;

Socket g_scpiSocket(AF_INET6, SOCK_STREAM, IPPROTO_TCP);
Socket g_dataSocket(AF_INET6, SOCK_STREAM, IPPROTO_TCP);

#ifdef _WIN32
BOOL WINAPI OnQuit(DWORD signal);
#else
void OnQuit(int signal);
#endif

int main(int argc, char* argv[])
{
	//Global settings
	Severity console_verbosity = Severity::NOTICE;
	g_series = 0;

	//Parse command-line arguments
	uint16_t scpi_port = 5025;
	uint16_t waveform_port = 5026;
	for(int i=1; i<argc; i++)
	{
		string s(argv[i]);

		//Let the logger eat its args first
		if(ParseLoggerArguments(i, argc, argv, console_verbosity))
			continue;

		if(s == "--help")
		{
			help();
			return 0;
		}

		else if(s == "--series")
		{
			if(i+1 < argc)
			{
				string tmp(argv[++i]);
				g_series = tmp[0] - '0';
				if( !( (g_series==2) || (g_series==3) || (g_series==4) || (g_series==5) || (g_series==6) ) )
					g_series = 0;
			}
		}

		else if(s == "--scpi-port")
		{
			if(i+1 < argc)
				scpi_port = atoi(argv[++i]);
		}

		else if(s == "--waveform-port")
		{
			if(i+1 < argc)
				waveform_port = atoi(argv[++i]);
		}

		else
		{
			fprintf(stderr, "Unrecognized command-line argument \"%s\", use --help\n", s.c_str());
			return 1;
		}
	}

	//Set up logging
	//TODO: fix color on Windows
	g_log_sinks.emplace(g_log_sinks.begin(), new ColoredSTDLogSink(console_verbosity));
	//g_log_sinks.emplace(g_log_sinks.begin(), new STDLogSink(console_verbosity));

	//For now, open the first instrument we can find.
	//TODO: implement device selection logic
	PICO_INFO status = PICO_NOT_FOUND;
	switch(g_series)
	{
		case 0:
		{
			status = Open2000();
			if(status == 0)
				break;
			status = Open3000();
			if(status == 0)
				break;
			status = Open4000();
			if(status == 0)
				break;
			status = Open5000();
			if(status == 0)
				break;
			status = Open6000();
			break;
		}
		case 2:
		{
			status = Open2000();
			break;
		}
		case 3:
		{
			status = Open3000();
			break;
		}
		case 4:
		{
			status = Open4000();
			break;
		}
		case 5:
		{
			status = Open5000();
			break;
		}
		case 6:
		{
			status = Open6000();
			break;
		}
	}

	if(PICO_OK != status)
	{
		LogError("Failed to open unit (code %d)\n", status);
		return 1;
	}

	//See what we got
	{
		LogIndenter li;

		char buf[128];
		int16_t required = 0;
		if( (g_pico_type == PICO2000) || (g_pico_type == PICO3000) )
		{
			status = picoGetUnitInfo2(g_hScope, (int8_t*)buf, sizeof(buf), PICO_DRIVER_VERSION);
			if(status == PICO_OK)
				LogVerbose("Driver version:   %s\n", buf);

			status = picoGetUnitInfo2(g_hScope, (int8_t*)buf, sizeof(buf), PICO_USB_VERSION);
			if(status == PICO_OK)
				LogVerbose("USB version:      %s\n", buf);

			status = picoGetUnitInfo2(g_hScope, (int8_t*)buf, sizeof(buf), PICO_HARDWARE_VERSION);
			if(status == PICO_OK)
				LogVerbose("Hardware version: %s\n", buf);

			status = picoGetUnitInfo2(g_hScope, (int8_t*)buf, sizeof(buf), PICO_VARIANT_INFO);
			if(status == PICO_OK)
			{
				LogVerbose("Variant info:     %s\n", buf);
				g_model = buf;
			}

			status = picoGetUnitInfo2(g_hScope, (int8_t*)buf, sizeof(buf), PICO_BATCH_AND_SERIAL);
			if(status == PICO_OK)
			{
				LogVerbose("Batch/serial:     %s\n", buf);
				g_serial = buf;
			}

			status = picoGetUnitInfo2(g_hScope, (int8_t*)buf, sizeof(buf), PICO_CAL_DATE);
			if(status == PICO_OK)
				LogVerbose("Cal date:         %s\n", buf);

			status = picoGetUnitInfo2(g_hScope, (int8_t*)buf, sizeof(buf), PS2000_KERNEL_DRIVER_VERSION);
			if(status == PICO_OK)
				LogVerbose("Kernel ver:       %s\n", buf);
		}
		else if( (g_pico_type == PICO5000) || (g_pico_type == PICO4000) )
		{
			status = picoGetUnitInfo(g_hScope, (int8_t*)buf, sizeof(buf), &required, PICO_DRIVER_VERSION);
			if(status == PICO_OK)
				LogVerbose("Driver version:   %s\n", buf);

			status = picoGetUnitInfo(g_hScope, (int8_t*)buf, sizeof(buf), &required, PICO_USB_VERSION);
			if(status == PICO_OK)
				LogVerbose("USB version:      %s\n", buf);

			status = picoGetUnitInfo(g_hScope, (int8_t*)buf, sizeof(buf), &required, PICO_HARDWARE_VERSION);
			if(status == PICO_OK)
				LogVerbose("Hardware version: %s\n", buf);

			status = picoGetUnitInfo(g_hScope, (int8_t*)buf, sizeof(buf), &required, PICO_VARIANT_INFO);
			if(status == PICO_OK)
			{
				LogVerbose("Variant info:     %s\n", buf);
				g_model = buf;
			}

			status = picoGetUnitInfo(g_hScope, (int8_t*)buf, sizeof(buf), &required, PICO_BATCH_AND_SERIAL);
			if(status == PICO_OK)
			{
				LogVerbose("Batch/serial:     %s\n", buf);
				g_serial = buf;
			}

			status = picoGetUnitInfo(g_hScope, (int8_t*)buf, sizeof(buf), &required, PICO_CAL_DATE);
			if(status == PICO_OK)
				LogVerbose("Cal date:         %s\n", buf);

			status = picoGetUnitInfo(g_hScope, (int8_t*)buf, sizeof(buf), &required, PICO_KERNEL_VERSION);
			if(status == PICO_OK)
				LogVerbose("Kernel ver:       %s\n", buf);
		}
		else
		{
			status = picoGetUnitInfo(g_hScope, (int8_t*)buf, sizeof(buf), &required, PICO_DRIVER_VERSION);
			if(status == PICO_OK)
				LogVerbose("Driver version:   %s\n", buf);

			status = picoGetUnitInfo(g_hScope, (int8_t*)buf, sizeof(buf), &required, PICO_USB_VERSION);
			if(status == PICO_OK)
				LogVerbose("USB version:      %s\n", buf);

			status = picoGetUnitInfo(g_hScope, (int8_t*)buf, sizeof(buf), &required, PICO_HARDWARE_VERSION);
			if(status == PICO_OK)
				LogVerbose("Hardware version: %s\n", buf);

			status = picoGetUnitInfo(g_hScope, (int8_t*)buf, sizeof(buf), &required, PICO_VARIANT_INFO);
			if(status == PICO_OK)
			{
				LogVerbose("Variant info:     %s\n", buf);
				g_model = buf;
			}

			status = picoGetUnitInfo(g_hScope, (int8_t*)buf, sizeof(buf), &required, PICO_BATCH_AND_SERIAL);
			if(status == PICO_OK)
			{
				LogVerbose("Batch/serial:     %s\n", buf);
				g_serial = buf;
			}

			status = picoGetUnitInfo(g_hScope, (int8_t*)buf, sizeof(buf), &required, PICO_CAL_DATE);
			if(status == PICO_OK)
				LogVerbose("Cal date:         %s\n", buf);

			status = picoGetUnitInfo(g_hScope, (int8_t*)buf, sizeof(buf), &required, PICO_KERNEL_VERSION);
			if(status == PICO_OK)
				LogVerbose("Kernel ver:       %s\n", buf);

			status = picoGetUnitInfo(g_hScope, (int8_t*)buf, sizeof(buf), &required, PICO_DIGITAL_HARDWARE_VERSION);
			if(status == PICO_OK)
				LogVerbose("Digital HW ver:   %s\n", buf);

			status = picoGetUnitInfo(g_hScope, (int8_t*)buf, sizeof(buf), &required, PICO_ANALOGUE_HARDWARE_VERSION);
			if(status == PICO_OK)
				LogVerbose("Analog HW ver:    %s\n", buf);

			status = picoGetUnitInfo(g_hScope, (int8_t*)buf, sizeof(buf), &required, PICO_FIRMWARE_VERSION_1);
			if(status == PICO_OK)
			{
				LogVerbose("FW ver 1:         %s\n", buf);
				g_fwver = buf;
			}

			status = picoGetUnitInfo(g_hScope, (int8_t*)buf, sizeof(buf), &required, PICO_FIRMWARE_VERSION_2);
			if(status == PICO_OK)
				LogVerbose("FW ver 2:         %s\n", buf);

			status = picoGetUnitInfo(g_hScope, (int8_t*)buf, sizeof(buf), &required, PICO_FIRMWARE_VERSION_3);
			if(status == PICO_OK)
				LogVerbose("FW ver 3:         %s\n", buf);

			status = picoGetUnitInfo(g_hScope, (int8_t*)buf, sizeof(buf), &required, PICO_FRONT_PANEL_FIRMWARE_VERSION);
			if(status == PICO_OK)
				LogVerbose("Front panel FW:   %s\n", buf);

			status = picoGetUnitInfo(g_hScope, (int8_t*)buf, sizeof(buf), &required, PICO_MAC_ADDRESS);
			if(status == PICO_OK)
				LogVerbose("MAC address:      %s\n", buf);

			status = picoGetUnitInfo(g_hScope, (int8_t*)buf, sizeof(buf), &required, PICO_DRIVER_PATH);
			if(status == PICO_OK)
				LogVerbose("Driver path:      %s\n", buf);

			status = picoGetUnitInfo(g_hScope, (int8_t*)buf, sizeof(buf), &required, PICO_SHADOW_CAL);
			if(status == PICO_OK)
				LogVerbose("Shadow cal:       %s\n", buf);

			status = picoGetUnitInfo(g_hScope, (int8_t*)buf, sizeof(buf), &required, PICO_IPP_VERSION);
			if(status == PICO_OK)
				LogVerbose("IPP version:      %s\n", buf);
		}
	}
	LogNotice("Successfully opened instrument %s (%s) on ports %i, %i\n", g_model.c_str(), g_serial.c_str(), scpi_port, waveform_port);

	//Limit to two channels only while on USB power
	if(limitChannels)
		g_numChannels = '2' - '0';
	else
		g_numChannels = g_model[1] - '0';

	//Initial channel state setup
	for(size_t i=0; i<g_numChannels; i++)
	{
		switch(g_pico_type)
		{
			case PICO2000:
				ps2000_set_channel(g_hScope, (PS2000_CHANNEL)i, 0, 1, PS2000_1V);
				break;
			case PICO2000A:
				ps2000aSetChannel(g_hScope, (PS2000A_CHANNEL)i, 0, PS2000A_DC, PS2000A_1V, 0.0f);
				break;
			case PICO3000:
				ps3000_set_channel(g_hScope, (PS3000_CHANNEL)i, 0, 1, PS3000_1V);
				break;
			case PICO3000A:
				ps3000aSetChannel(g_hScope, (PS3000A_CHANNEL)i, 0, PS3000A_DC, PS3000A_1V, 0.0f);
				break;
			case PICO4000:
				ps4000SetChannel(g_hScope, (PS4000_CHANNEL)i, 0, 1, PS4000_1V);
				break;
			case PICO4000A:
				ps4000aSetChannel(g_hScope, (PS4000A_CHANNEL)i, 0, PS4000A_DC, PICO_X1_PROBE_1V, 0.0f);
				break;
			case PICO5000:
				ps5000SetChannel(g_hScope, (PS5000_CHANNEL)i, 0, 1, PS5000_1V);
				break;
			case PICO5000A:
				ps5000aSetChannel(g_hScope, (PS5000A_CHANNEL)i, 0, PS5000A_DC, PS5000A_1V, 0.0f);
				break;
			case PICO6000:
				ps6000SetChannel(g_hScope, (PS6000_CHANNEL)i, 0, PS6000_DC_1M, PS6000_1V, 0.0f, PS6000_BW_FULL);
				break;
			case PICO6000A:
				ps6000aSetChannelOff(g_hScope, (PICO_CHANNEL)i);
				break;
			case PICOPSOSPA:
				psospaSetChannelOff(g_hScope, (PICO_CHANNEL)i);
				break;
		}
	}

	//Initialize analog channels
	for(size_t i=0; i<g_numChannels; i++)
	{
		g_channelOn[i] = false;
		g_coupling[i] = PICO_DC;
		g_range[i] = PICO_X1_PROBE_1V;
		g_range_psospa[i] = PICO_X1_PROBE_NV;
		g_range_2000a[i] = PS2000A_1V;
		g_range_3000e[i] = 1000000000;
		g_range_3000a[i] = PS3000A_1V;
		g_range_4000a[i] = PS4000A_1V;
		g_range_5000a[i] = PS5000A_1V;
		g_range_2000[i] = PS2000_1V;
		g_range_3000[i] = PS3000_1V;
		g_range_4000[i] = PS4000_1V;
		g_range_5000[i] = PS5000_1V;
		g_range_6000[i] = PS6000_1V;
		g_offset[i] = 0;
		g_bandwidth[i] = PICO_BW_FULL;
		g_bandwidth_3000a[i] = PS3000A_BW_FULL;
		g_bandwidth_4000a[i] = PS4000A_BW_FULL;
		g_bandwidth_5000a[i] = PS5000A_BW_FULL;
		g_bandwidth_6000[i] = PS6000_BW_FULL;
		g_bandwidth_4000[i] = 0;
	}

	//Figure out digital channel configuration
	switch(g_series)
	{
		//No digital options for 4000 series.
		
		case 2:
		case 3:
		case 5:
			/* Model 3abcdMSO with
				a=4 chan, b=0(unknown) c=6(bandwidth 6=200MHz) d=D(revision D)
				MSO if MSO option is available
				example 3406DMSO (full option)
			*/
			/* Model 5abcdMSO with
				a=4 chan, b=4(flexres up to 16bit) c=4(bandwidth 4=200MHz) d=D(revision D)
				MSO if MSO option is available
				example 5444DMSO (full option)
			*/
			if(g_model.find("MSO") != string::npos)
			{
				g_numDigitalPods = 2;
			}
			else
			{
				g_numDigitalPods = 0;
			}
			break;

		case 6:
			g_numDigitalPods = 2;
			break;

		default:
			g_numDigitalPods = 0;
	}

	for(size_t i=0; i<g_numDigitalPods; i++)
	{
		g_msoPodEnabled[i] = false;
		for(size_t j=0; j<8; j++)
			g_msoPodThreshold[i][j] = 0;
		g_msoHysteresis[i] = PICO_NORMAL_100MV;
	}

	//Push initial trigger config
	UpdateTrigger();

	//Set up signal handlers
#ifdef _WIN32
	SetConsoleCtrlHandler(OnQuit, TRUE);
#else
	signal(SIGINT, OnQuit);
	signal(SIGPIPE, SIG_IGN);
#endif

	//Configure the data plane socket
	g_dataSocket.Bind(waveform_port);
	g_dataSocket.Listen();

	//Launch the control plane socket server
	g_scpiSocket.Bind(scpi_port);
	g_scpiSocket.Listen();

	while(true)
	{
		Socket scpiClient = g_scpiSocket.Accept();
		if(!scpiClient.IsValid())
			break;

		//Create a server object for this connection
		PicoSCPIServer server(scpiClient.Detach());

		//Launch the data-plane thread
		thread dataThread(WaveformServerThread);

		//Process connections on the socket
		server.MainLoop();

		g_waveformThreadQuit = true;
		dataThread.join();
		g_waveformThreadQuit = false;
	}

	//Done
	switch(g_pico_type)
	{
		case PICO2000:
			ps2000_close_unit(g_hScope);
			break;
		case PICO2000A:
			ps2000aCloseUnit(g_hScope);
			break;
		case PICO3000:
			ps3000_close_unit(g_hScope);
			break;
		case PICO3000A:
			ps3000aCloseUnit(g_hScope);
			break;
		case PICO4000:
			ps4000CloseUnit(g_hScope);
			break;
		case PICO4000A:
			ps4000aCloseUnit(g_hScope);
			break;
		case PICO5000:
			ps5000CloseUnit(g_hScope);
			break;
		case PICO5000A:
			ps5000aCloseUnit(g_hScope);
			break;
		case PICO6000:
			ps6000CloseUnit(g_hScope);
			break;
		case PICO6000A:
			ps6000aCloseUnit(g_hScope);
			break;
		case PICOPSOSPA:
			psospaCloseUnit(g_hScope);
			break;
	}

	return 0;
}

#ifdef _WIN32
BOOL WINAPI OnQuit(DWORD signal)
{
	(void)signal;
#else
void OnQuit(int /*signal*/)
{
#endif
	LogNotice("Shutting down...\n");

	lock_guard<mutex> lock(g_mutex);
	switch(g_pico_type)
	{
		case PICO2000:
			ps2000_close_unit(g_hScope);
			break;
		case PICO2000A:
			ps2000aCloseUnit(g_hScope);
			break;
		case PICO3000:
			ps3000_close_unit(g_hScope);
			break;
		case PICO3000A:
			ps3000aCloseUnit(g_hScope);
			break;
		case PICO4000:
			ps4000CloseUnit(g_hScope);
			break;
		case PICO4000A:
			ps4000aCloseUnit(g_hScope);
			break;
		case PICO5000:
			ps5000CloseUnit(g_hScope);
			break;
		case PICO5000A:
			ps5000aCloseUnit(g_hScope);
			break;
		case PICO6000:
			ps6000CloseUnit(g_hScope);
			break;
		case PICO6000A:
			ps6000aCloseUnit(g_hScope);
			break;
		case PICOPSOSPA:
			psospaCloseUnit(g_hScope);
			break;
	}
	exit(0);
}

PICO_INFO Open2000()
{
	LogNotice("Looking for a PicoScope 2000 series instrument to open...\n");
	PICO_INFO status = ps2000aOpenUnit(&g_hScope, NULL);
	if(status == PICO_OK)
	{
		g_series = 2;
		g_pico_type = PICO2000A;
		picoGetUnitInfo = ps2000aGetUnitInfo;
		return status;
	}

	g_hScope = ps2000_open_unit();
	if(g_hScope > 0)
	{
		g_series = 2;
		g_pico_type = PICO2000;
		picoGetUnitInfo2 = ps2000_get_unit_info;
		return PICO_OK;
	}
	return status;
}
PICO_INFO Open3000()
{
	LogNotice("Looking for a PicoScope 3000 series instrument to open...\n");
	PICO_INFO status = ps3000aOpenUnit(&g_hScope, NULL);
	if(status == PICO_POWER_SUPPLY_NOT_CONNECTED)
	{
		// switch to USB power
		// TODO: maybe require the user to specify this is ok
		limitChannels = true;
		LogNotice("Switching to USB power...\n");
		status = ps3000aChangePowerSource(g_hScope, PICO_POWER_SUPPLY_NOT_CONNECTED);
	}
	if(status == PICO_OK)
	{
		g_series = 3;
		g_pico_type = PICO3000A;
		picoGetUnitInfo = ps3000aGetUnitInfo;
		return status;
	}

	status = psospaOpenUnit(&g_hScope, NULL, PICO_DR_8BIT, NULL);
	if(status == PICO_OK)
	{
		g_series = 3;
		g_pico_type = PICOPSOSPA;
		picoGetUnitInfo = psospaGetUnitInfo;
		return status;
	}

	g_hScope = ps3000_open_unit();
	if(g_hScope > 0)
	{
		g_series = 3;
		g_pico_type = PICO3000;
		picoGetUnitInfo2 = ps3000_get_unit_info;
		return PICO_OK;
	}
	return status;
}

PICO_INFO Open4000()
{
	LogNotice("Looking for a PicoScope 4000 series instrument to open...\n");
	PICO_INFO status = ps4000aOpenUnit(&g_hScope, NULL);
	if(status == PICO_POWER_SUPPLY_NOT_CONNECTED)
	{
		// switch to USB power
		// only applies to model 4444
		limitChannels = true;
		LogNotice("Switching to USB power...\n");
		status = ps4000aChangePowerSource(g_hScope, PICO_POWER_SUPPLY_NOT_CONNECTED);
	}
	if(status == PICO_OK)
	{
		g_series = 4;
		g_pico_type = PICO4000A;
		picoGetUnitInfo = ps4000aGetUnitInfo;
		return status;
	}

	status = ps4000OpenUnit(&g_hScope);
	if(status == PICO_OK)
	{
		g_series = 4;
		g_pico_type = PICO4000;
		picoGetUnitInfo = ps4000GetUnitInfo;
	}
	return status;
}

PICO_INFO Open5000()
{
	LogNotice("Looking for a PicoScope 5000 series instrument to open...\n");
	PICO_INFO status = ps5000aOpenUnit(&g_hScope, NULL, PS5000A_DR_8BIT);
	if( (status == PICO_POWER_SUPPLY_NOT_CONNECTED) or (status == PICO_USB3_0_DEVICE_NON_USB3_0_PORT) )
	{
		// switch to USB power
		// TODO: maybe require the user to specify this is ok
		limitChannels = true;
		LogNotice("Switching to USB power...\n");
		status = ps5000aChangePowerSource(g_hScope, PICO_POWER_SUPPLY_NOT_CONNECTED);
	}
	if(status == PICO_OK)
	{
		g_series = 5;
		g_pico_type = PICO5000A;
		picoGetUnitInfo = ps5000aGetUnitInfo;
		return status;
	}

	status = ps5000OpenUnit(&g_hScope);
	if(status == PICO_OK)
	{
		g_series = 5;
		g_pico_type = PICO5000;
		picoGetUnitInfo = ps5000GetUnitInfo;
	}
	return status;
}
PICO_INFO Open6000()
{
	LogNotice("Looking for a PicoScope 6000 series instrument to open...\n");
	PICO_INFO status = ps6000aOpenUnit(&g_hScope, NULL, PICO_DR_8BIT);
	if(status == PICO_OK)
	{
		g_series = 6;
		g_pico_type = PICO6000A;
		picoGetUnitInfo = ps6000aGetUnitInfo;
		return status;
	}

	status = ps6000OpenUnit(&g_hScope, NULL);
	if(status == PICO_OK)
	{
		g_series = 6;
		g_pico_type = PICO6000;
		picoGetUnitInfo = ps6000GetUnitInfo;
	}
	return status;
}
