/*
 *
 *   ===================================================
 *       CropDrop Bot (CB) Theme [eYRC 2025-26]
 *   ===================================================
 *
 *  This script is intended to be a Boilerplate for
 *  Task 2b of CropDrop Bot (CB) Theme [eYRC 2025-26].
 *
 *  Filename:        Task2b.c
 *  Created:         19/08/2025
 *  Last Modified:   21/10/2025
 *  Author:          Team members Name
 *  Team ID:         [ CB_xxxx ]
 *
 *  This software is made available on an "AS IS WHERE IS BASIS".
 *  Licensee/end user indemnifies and will keep e-Yantra indemnified from
 *  any and all claim(s) that emanate from the use of the Software or
 *  breach of the terms of this agreement.
 *
 *  e-Yantra - An MHRD project under National Mission on Education using ICT (NMEICT)
 *
 **********************************************
 */

#ifdef _WIN32
#define WINVER 0x0600
#define _WIN32_WINNT 0x0600
#include <winsock2.h>
#include <ws2tcpip.h>
#endif

#include "coppeliasim_client.h"
#include <sys/time.h>
#include <math.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>

int box_count = 0;
bool isbox = false;
char box_color[16] = "None"; // changed from string to char array for C compatibility

// Global client instance for socket communication
SocketClient client;

// ----------------------
// Forward declarations
// ----------------------
void *control_loop(void *arg);
void handlebox(SocketClient *c);
const char *color(SocketClient *c);

/**
 * @brief Establishes connection to the CoppeliaSim server
 */
int connect_to_server(SocketClient *c, const char *ip, int port)
{
#ifdef _WIN32
    WSADATA wsa;
    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0)
    {
        printf("WSAStartup failed\n");
        return 0;
    }
#endif

    c->sock = socket(AF_INET, SOCK_STREAM, 0);
    if (c->sock < 0)
    {
        printf("Socket creation failed\n");
        return 0;
    }

    struct sockaddr_in serv_addr;
    memset(&serv_addr, 0, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(port);
    inet_pton(AF_INET, ip, &serv_addr.sin_addr);

    if (connect(c->sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
    {
        printf("Connection failed\n");
        CLOSESOCKET(c->sock);
#ifdef _WIN32
        WSACleanup();
#endif
        return 0;
    }

    c->running = true;

#ifdef _WIN32
    c->recv_thread = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)receive_loop, c, 0, NULL);
#else
    pthread_create(&c->recv_thread, NULL, receive_loop, c);
#endif

    return 1;
}

/**
 * @brief Get current time in seconds
 */
double get_current_time()
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec + tv.tv_usec / 1000000.0;
}

/**
 * @brief Detect color based on sensor values (keeping your color swap logic as-is)
 */

/**
 * @brief Handles box detection and sending color
 */

/**
 * @brief Main control loop for robot behavior
 */
void *control_loop(void *arg)
{
    SocketClient *c = (SocketClient *)arg;

    // -----------------------------
    // PID constants
    // -----------------------------
    float Kp = 7.5f;
    float Ki = 0.001f;
    float Kd = 4.0f;

    float integral = 0.0f;
    float prev_error = 0.0f;

    const float base_speed = 3.0f;
    const float max_speed = 3.0f;

    // Adaptive line mode variables
    bool white_line_mode = true;
    float avg_brightness = 0.0f;
    const float switch_thresh_high = 0.6f;
    const float switch_thresh_low = 0.4f;
    const float alpha = 0.1f;

    int nodecount = 0;
    SLEEP(1000);

    while (c->running)
    {
        float raw[5];
        for (int i = 0; i < 5; i++)
            raw[i] = c->line_sensors[i];

        // -----------------------------
        // Adaptive white/black line
        // -----------------------------
        float brightness_sum = raw[0] + raw[1] + raw[2] + raw[3] + raw[4];
        float brightness_avg = brightness_sum / 5.0f;
        avg_brightness = alpha * brightness_avg + (1 - alpha) * avg_brightness;

        if (white_line_mode && avg_brightness > switch_thresh_high)
            white_line_mode = false;
        else if (!white_line_mode && avg_brightness < switch_thresh_low)
            white_line_mode = true;

        float ir[5];
        if (white_line_mode)
        {
            for (int i = 0; i < 5; i++)
                ir[i] = 1 - raw[i];
        }
        else
        {
            for (int i = 0; i < 5; i++)
                ir[i] = raw[i];
        }

        // -----------------------------
        // PID error calculation
        // -----------------------------
        int weights[5] = {-2, -1, 0, 1, 2};
        float sum_ir = ir[0] + ir[1] + ir[2] + ir[3] + ir[4];
        if (sum_ir == 0) sum_ir = 1;

        float error = (weights[0]*ir[0] + weights[1]*ir[1] + weights[2]*ir[2] +
                       weights[3]*ir[3] + weights[4]*ir[4]) / sum_ir;

        integral += error;
        float derivative = error - prev_error;
        prev_error = error;

        float correction = Kp*error + Ki*integral + Kd*derivative;

        float left_speed = base_speed - correction;
        float right_speed = base_speed + correction;

        // Speed limits
        if (left_speed > max_speed) left_speed = max_speed;
        if (left_speed < -max_speed) left_speed = -max_speed;
        if (right_speed > max_speed) right_speed = max_speed;
        if (right_speed < -max_speed) right_speed = -max_speed;

        // -----------------------------
        // Box handling
        // -----------------------------
        if (c->proximity_distance < 0.5 && !isbox)
        {
            if (c->color_g > c->color_b && c->color_g > c->color_r)
                strcpy(box_color, "red");
            else if (c->color_b > c->color_g && c->color_b > c->color_r)
                strcpy(box_color, "green");
            else if (c->color_r > c->color_b && c->color_r > c->color_g)
                strcpy(box_color, "blue");
            else
                strcpy(box_color, "None");

            if (strcmp(box_color, "None") != 0)
            {
                printf("Detected Color: %s\n", box_color);
                send_color(c, box_color);
                box_count++;
                printf("%d\n", box_count);
                if (box_count == 3)
                    pick_box(c);
                isbox = true;
            }
        }
        else if (c->proximity_distance > 0.5)
        {
            isbox = false;
        }

        // -----------------------------
        // Node logic (only on black line)
        // -----------------------------
        if (!white_line_mode && ir[2] <= 0.4 && ir[3] <= 0.4 && ir[4] <= 0.4)
        {
            nodecount++;
            set_motor(c, 0.0, 0.0);
            SLEEP(1000);
            if (nodecount == 1)
            {
                if (strcmp(box_color, "red") == 0 || strcmp(box_color, "green") == 0)
                {
                    set_motor(c, 0.0, 3.0); // turn right
                    SLEEP(1300);
                }
                else if (strcmp(box_color, "blue") == 0)
                {
                    set_motor(c, 3.0, 0.0); // turn right
                    SLEEP(1300);
                }
            }
            else if (nodecount == 2)
            {
                set_motor(c, 0.0, 0.0); // stop
                SLEEP(1000);
                drop_box(c);
            }
        }
        // printf("nodecount %d\n",nodecount);
        set_motor(c, left_speed, right_speed);
        SLEEP(1);
    }

    return NULL;
}


/**
 * @brief Main function
 */
int main()
{
    if (!connect_to_server(&client, "127.0.0.1", 50002))
    {
        printf("Failed to connect to CoppeliaSim server. Make sure:\n");
        printf("1. CoppeliaSim is running\n");
        printf("2. The simulation scene is loaded\n");
        printf("3. The ZMQ remote API is enabled on port 50002\n");
        return -1;
    }

    printf("✅ Connected to CoppeliaSim server!\n");
    printf("🚀 Starting control thread...\n");

#ifdef _WIN32
    HANDLE control_thread = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)control_loop, &client, 0, NULL);
#else
    pthread_t control_thread;
    pthread_create(&control_thread, NULL, control_loop, &client);
#endif

    printf("📡 Monitoring sensor data... (Press Ctrl+C to exit)\n");
    while (1)
    {
        SLEEP(100);
    }

    printf("Disconnecting...\n");
    disconnect(&client);
    return 0;
}
