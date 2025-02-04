#include <stdio.h>      // Fornisce funzioni di input/output (es. printf, perror)
#include <string.h>     // Per funzioni di manipolazione delle stringhe (es. memset)
#include <fcntl.h>      // Per lavorare con file descriptor, modalità e flag (es. open)
#include <termios.h>    // Consente di configurare e controllare porte seriali
#include <unistd.h>     // Fornisce funzioni per la gestione dei file e dei processi (es. close, read)
#include <signal.h>     // Used to terminate the program
#include <stdlib.h>     // Used for some utilities (es. exit)

#define SERIAL_PORT "/dev/ttyAMA0"      // serial port's name
#define DATA_BUFFER_SIZE 50             // buffer size for incoming data
#define COMMAND_BUFFER_SIZE 10          // buffer size for commands to send

// Global variable
int serial_fd;

void handle_signal(int signal) {
    printf("\nTerminating...\n");
    close(serial_fd);           // Close serial port
    exit(0);                    // End the program
}

int main() {
    //-----------------------------------------------------------
    // Configuration part
    //-----------------------------------------------------------

    // Configuration: signal
    signal(SIGINT, handle_signal);

    // Configuration variable
    struct termios tty;

    // trying to open the serial port
    serial_fd = open(SERIAL_PORT, O_RDWR | O_NOCTTY | O_NDELAY);
    if (serial_fd == -1) {
        printf("Unable to open serial port %s\n", SERIAL_PORT);
        return 1;
    }

    // remove non blocking mode
    fcntl(serial_fd, F_SETFL, 0);

    // Get current port configuration
    memset(&tty, 0, sizeof(tty));
    if (tcgetattr(serial_fd, &tty) != 0) {
        printf("Unable to read current port configuration\n");
        close(serial_fd);
        return 1;
    }

    // Confiration of baud rate
    cfsetispeed(&tty, B19200); // input velocity
    cfsetospeed(&tty, B19200); // output velocity

    // Confiration of word length, parity, and stop bit
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8 bit
    tty.c_cflag |= PARENB;                      // enable parity
    tty.c_cflag |= PARODD;                      // odd parity
    tty.c_cflag &= ~CSTOPB;                     // 1 stop bit
    tty.c_cflag &= ~CRTSCTS;                    // Disable hardware control

    tty.c_lflag = 0;        // Raw mode
    tty.c_cc[VMIN]  = 1;    // At least 1 byte needed

    // Set new configuration
    if (tcsetattr(serial_fd, TCSANOW, &tty) != 0) {
        printf("Unable to set new port configuration\n");
        close(serial_fd);
        return 1;
    }

    //-----------------------------------------------------------
    // main part
    //-----------------------------------------------------------

    printf("Port %s ready. Listening...\n", SERIAL_PORT);

    // main variables
    char buffer[DATA_BUFFER_SIZE];                      // Buffer for incoming data
    char command[COMMAND_BUFFER_SIZE] = "inc 500ms\0";  // Command to be sent
    int i = 0;                                          // readings chunks counter

    while (1) {
        ++i;
        memset(buffer, 0, DATA_BUFFER_SIZE);

        // Read incoming data
        int bytes_read = read(serial_fd, buffer, sizeof(buffer) - 1);   //keep space for '\0'
        if (bytes_read > 0) {
            buffer[bytes_read] = '\0';  // Set the string end
            printf("(Data chunk received: %s)\n", buffer);
        } else if (bytes_read < 0) {
            printf("Reading failed\n");
        }
        
        // Send a command to the other board
        if (i%10==0) {
            int bytes_written = write(serial_fd, command, COMMAND_BUFFER_SIZE);

            if (bytes_written < 0) {
                printf("Failed to send message to the other board\n");
            } else {
                printf("(Command sent: %s)\n", command);
            }
        }
        fflush(stdout);
    }
}
