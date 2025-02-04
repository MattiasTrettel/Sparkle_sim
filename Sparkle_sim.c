#include <stdio.h>      // Fornisce funzioni di input/output (es. printf, perror)
#include <stdint.h>     // Per uint16
#include <string.h>     // Per funzioni di manipolazione delle stringhe (es. memset)
#include <fcntl.h>      // Per lavorare con file descriptor, modalità e flag (es. open)
#include <termios.h>    // Consente di configurare e controllare porte seriali
#include <unistd.h>     // Fornisce funzioni per la gestione dei file e dei processi (es. close, read)
#include <signal.h>     // Usato per terminare il programma
#include <stdlib.h>     // Usato per alcune utilità (es. exit)
#include <pthread.h>    // Per la gestione dei thread
#include <time.h>       // Per il timer del thread di trasmissione
#include <stdbool.h>    //bool variables

// Global variables
pthread_mutex_t log_mutex = PTHREAD_MUTEX_INITIALIZER;  //for writing on files
pthread_mutex_t debug_mutex = PTHREAD_MUTEX_INITIALIZER;
volatile int running = 1;                               //to stop threads
int serial_fd;                                          //serial port file descriptor
FILE* log_file;                                         //log file pointer
FILE* debug_file; 
//-----------------------------------------------------------
// Packages menagment
//-----------------------------------------------------------

// Costanti del pacchetto
#define MAGIC_NUM_1 0xA5
#define MAGIC_NUM_2 0xA5
#define MAGIC_NUM_3 0xA5
#define MAGIC_NUM_4 0xA5

#define PAYLOAD_ID  0xFF        // ID fisso fornito
#define PACKET_TYPE_TC_GENERIC  0x00
#define PACKET_TYPE_TC_TIME     0x0E
#define PACKET_TYPE_TC_SYN      0x0F

#define PACKET_TYPE_TM_HK   0x01  // Housekeeping
#define PACKET_TYPE_TM_SC   0x02  // Science
#define PACKET_TYPE_TM_AN   0x08  // Ancillary
#define PACKET_TYPE_TM_ACK  0x03  // Acknowledge
#define PACKET_TYPE_TM_NAK  0x04  // Not Acknowledge

#define CRC_POLY 0x1021        // Polinomio CRC: X^16 + X^12 + X^5 + 1
#define CRC_INIT 0xFFFF        // Valore iniziale CRC

//#define MAX_TC_PKG_SIZE 256   //arbitrary set //useless -> internal check

// Structure Header
typedef struct{
    uint8_t magic[4];         // Magic number
    uint8_t payload_id;       // ID del payload
    uint8_t packet_type;      // Tipo di pacchetto
    uint16_t cargo_length;    // Lunghezza del cargo
    uint16_t packet_counter;  // Contatore del pacchetto
} TC_Header;

typedef struct {
    uint8_t magic[4];      	// Magic number 0xA5A5A5A5
    uint8_t payload_id;    	// Payload ID
    uint8_t packet_type;   	// Package type
    uint16_t cargo_length; 	// Package length
    uint16_t packet_counter;// Package counter
    uint32_t coarse_time;  	// Time (seconds)
    uint8_t fine_time[3];  	// Time (fraction of second)
} TM_Header;

typedef struct {
    TM_Header header; 
    uint8_t* cargo;
    uint16_t crc;
} TM_Packet;

// Funzione per calcolare il CRC-16 CCITT
uint16_t calculate_crc(uint8_t *data, size_t length){
    uint16_t crc = CRC_INIT;                            // Remainder iniziale
    for (size_t i = 0; i < length; i++) {
        crc ^= (data[i] << 8);                          // XOR byte corrente nel CRC
        for (int bit = 0; bit < 8; bit++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ CRC_POLY;            // XOR con il polinomio
            } else {
                crc <<= 1;                              // Shift solo
            }
        }
    }
    return crc & 0xFFFF;                                // Ritorna un valore a 16 bit
}

int send_packet(uint8_t packet_type, uint8_t* cargo, uint16_t cargo_length, uint16_t* packet_counter) {
    // Header costruzione
    TC_Header header;
    memset(&header, 0, sizeof(TC_Header));    // Azzera la memoria

    header.magic[0] = MAGIC_NUM_1;
    header.magic[1] = MAGIC_NUM_2;
    header.magic[2] = MAGIC_NUM_3;
    header.magic[3] = MAGIC_NUM_4;

    header.payload_id = PAYLOAD_ID;             // ID del payload
    header.packet_type = packet_type;           // Tipo di pacchetto
    header.cargo_length = cargo_length;         // Lunghezza del cargo
    header.packet_counter = (*packet_counter);  // Contatore, incrementato


    // Preparazione del buffer dati
    size_t total_length = sizeof(TC_Header) + cargo_length + sizeof(uint16_t);  // Header + Cargo + CRC
    uint8_t buffer[total_length];                                               // Assume dimensione massima sufficiente

    memcpy(buffer, &header, sizeof(TC_Header));                                 // Copia l'header nel buffer
    if(cargo_length>0){
        memcpy(buffer + sizeof(TC_Header), cargo, cargo_length);                // Copia il cargo nel buffer
    }

    uint16_t crc = calculate_crc(buffer, sizeof(TC_Header) + cargo_length);     // Calcolo CRC
    memcpy(buffer + sizeof(TC_Header) + cargo_length, &crc, sizeof(crc));       // Aggiunge il CRC al buffer
    //-------------------------------------
    pthread_mutex_lock(&debug_mutex);
    fprintf(debug_file, "Sending command: ");
	for (int i = 0; i < total_length; i++) {
		fprintf(debug_file," %02X", buffer[i]);
	}
	fprintf(debug_file,"\n");
    fflush(debug_file);
    pthread_mutex_unlock(&debug_mutex);
    //-------------------------------------

    return write(serial_fd, buffer, total_length); 
}


//-----------------------------------------------------------
// Interface
//-----------------------------------------------------------

// Coordinate fisse per il layout
#define PROMPT_ROW 8
#define STATUS_ROW 13
#define RESPONSE_ROW 15
#define HISTORY_ROW 21

#define HISTORY_SIZE 10
#define MAX_COMMAND_LEN 20              // Include history command counter
#define COMMAND_BUFFER_SIZE 10          // Dimensione buffer per i comandi da inviare

#define MAX_NO_RESPONSE_TRYING 3
#define NO_RESPONSE_TIMER 5
#define MAX_NAK_TRYING 3
volatile uint8_t response = 0;

// Imposta il cursore in una posizione specifica
void move_cursor(int row, int col){
    printf("\033[%d;%dH", row, col);
    fflush(stdout);
}

// Disegna il layout iniziale
void draw_interface(){
    printf("\033[2J\033[H");
    printf("Sparkle Sim Tele-Command Interface\n\n");
    printf("TC available:\n");
    printf("   start\n");
    printf("   stop\n");
    printf("   sync\n\n");
    printf("Command to send?\n\n");
    printf("\n----------------------------------\n\n");
    printf("Current TC: none\n\n");
    printf("\n\n----------------------------------\n\n");
    printf("History (last 10 valid commands):");
    fflush(stdout);
}


void update_history(const char *command, char history[HISTORY_SIZE][MAX_COMMAND_LEN], uint16_t *packet_counter, int *pointer) {
    snprintf(history[*pointer], MAX_COMMAND_LEN, "(%i) %s", *packet_counter, command);

    move_cursor(HISTORY_ROW, 1);
    int display_count = (*packet_counter < HISTORY_SIZE) ? *packet_counter : HISTORY_SIZE;
    for (int i = 0; i < display_count; i++) {
        int idx = (*pointer - i + HISTORY_SIZE) % HISTORY_SIZE;
        printf("%s\033[K\n", history[idx]);
    }
    *pointer = (*pointer + 1) % HISTORY_SIZE;
}



// Processa il comando inserito
void process_command(const char *command, char history[HISTORY_SIZE][MAX_COMMAND_LEN], uint16_t* packet_counter, int* pointer, bool* sync){
    //move_cursor(STATUS_ROW, 13);
    if (strcmp(command, "start") == 0 || strcmp(command, "stop") == 0 || strcmp(command, "sync") == 0 || *sync==true) {
        uint8_t packet_type = PACKET_TYPE_TC_GENERIC;
        const char *cargo = NULL;
        static char sync_cargo[5];
        static uint32_t future_MET;
        if(*sync){
            snprintf((char *)command, MAX_COMMAND_LEN, "sync_pt2");
            packet_type = PACKET_TYPE_TC_SYN;
            cargo = "";
        }
        else if (strcmp(command, "start") == 0) {
            cargo = "Request TM";
        }else if (strcmp(command, "stop") == 0) {
            cargo = "Stop TM";
        }else if (strcmp(command, "sync") == 0) {
            snprintf((char *)command, MAX_COMMAND_LEN, "sync_pt1");
            packet_type = PACKET_TYPE_TC_TIME;
            future_MET = (uint32_t)time(NULL) + 5;
            memcpy(sync_cargo, &future_MET, 4);
            sync_cargo[4] = '\0';
            cargo = sync_cargo;
        }
        bool response_obtained=false;
        uint8_t no_response_counter = 0;
        uint8_t nak_counter = 0;
        response=0;
        while ((!response_obtained) && (no_response_counter < MAX_NO_RESPONSE_TRYING) && (nak_counter < MAX_NAK_TRYING)) {
            move_cursor(STATUS_ROW, 13);
            if(*sync){
                while((uint32_t)time(NULL) < future_MET);
            }
            int bytes_sent = send_packet(packet_type, (uint8_t *)cargo, strlen(cargo), packet_counter);

            if (bytes_sent < 0) {
                printf("Failed to send TC to the other board\n");
                fflush(stdout);
                return;
            }
            printf("%s\033[K\nSending... (%i bytes)\033[K\nResponse:\033[K", command, bytes_sent);
            fflush(stdout);
            move_cursor(RESPONSE_ROW, 11);

            time_t start_time = time(NULL);  

            while (response == 0 && (time(NULL) - start_time) < NO_RESPONSE_TIMER){
                usleep(10000);
            }
            if (response == 1) {
                printf("Acknowledge\033[K");
                fflush(stdout);
                response_obtained = true;
                if(packet_type==PACKET_TYPE_TC_TIME){
                    (*sync) = true;
                }
            } else if (response == 2) {
                printf("Not Acknowledge, retrying...\033[K");
                fflush(stdout);
                nak_counter++;
                sleep(1);
                continue;
            } else {
                printf("Timer elapsed, retrying...\033[K");
                fflush(stdout);
                no_response_counter++;
                sleep(1);
                continue;
            }
        }
        if (!response_obtained) {
            move_cursor(RESPONSE_ROW, 11);
            printf("Max trying reached, aborting...\033[K");
            fflush(stdout);
        } else {
            (*packet_counter)++;
            update_history(command, history, packet_counter, pointer);
        }
    } else {
        printf("Unknown\033[K\n\033[K\n\033[K");
    }
    fflush(stdout);
}


//-----------------------------------------------------------
// Serial port menagment
//-----------------------------------------------------------

#define SERIAL_PORT "/dev/ttyAMA0"      // Nome della porta seriale
#define MAX_TM_CARGO_SIZE 1103          // Dimensione buffer per i dati ricevuti
#define TM_HEADER_SIZE 17
#define READ_CHUNK_SIZE 16
#define LOG_FILE "Sparkle_log.txt"      // Nome del file di log
#define DEBUG_FILE "Sparkle_debug.txt"


void handle_signal(int signal) {
    running = 0; // Segnale per terminare i thread
    printf("\033[2J\033[H");
    printf("Terminating...\n");
    if (log_file) {
        fclose(log_file);  // Chiude il file di log
    }
    if(debug_file){
        fclose(debug_file);
    }
    close(serial_fd);       // Chiude la porta seriale
    exit(0);                // Termina il programma
}

void* receiver_thread(void* arg) {
    uint8_t buffer[MAX_TM_CARGO_SIZE + TM_HEADER_SIZE + 2 + READ_CHUNK_SIZE];  // Buffer per pacchetto completo (+3 = ultimo byte header & crc)
    memset(buffer, 0, sizeof(buffer));
    TM_Header header;

    int total_bytes=0;
    while (running) {
        //memset(buffer, 0, sizeof(buffer));

        bool head = false;
        bool cargo_len_bytes = false;
        while(!head){
            int bytes_read = read(serial_fd, buffer + total_bytes, READ_CHUNK_SIZE);
            if(bytes_read>0){
                total_bytes += bytes_read;
            }
            int pos_head=-1;
            for (int i = 0; i <= total_bytes - 4; i++) {
                if (buffer[i] == MAGIC_NUM_1 && buffer[i+1] == MAGIC_NUM_2 &&
                    buffer[i+2] == MAGIC_NUM_3 && buffer[i+3] == MAGIC_NUM_4) {
                    pos_head = i;
                    break;
                }
            }
            if(pos_head==-1){
                if (total_bytes + READ_CHUNK_SIZE > sizeof(buffer)) {
                    int shift = total_bytes + READ_CHUNK_SIZE - sizeof(buffer);
                    memmove(buffer, buffer + shift, total_bytes - shift);
                    total_bytes -= shift;
                }
                fprintf(log_file, "(Corrupted TM package (no header found))\n");
                continue;
            }
            else if(pos_head>0){
                memmove(buffer, buffer + pos_head, total_bytes - pos_head);  // Sposta i byte rimanenti a sinistra
                total_bytes -= pos_head;                                     // Aggiorna il conteggio dei byte validi
            }
            head = true;
        }
        while(!cargo_len_bytes){
            if(total_bytes < TM_HEADER_SIZE){
                int bytes_read = read(serial_fd, buffer + total_bytes, READ_CHUNK_SIZE);
                if(bytes_read>0){
                    total_bytes += bytes_read;
                }
            }
            else{
                memcpy(&header, buffer, TM_HEADER_SIZE);
                cargo_len_bytes=true;
            }
        }
        if(header.cargo_length>MAX_TM_CARGO_SIZE){
            fprintf(log_file, "(Corrupted TM package (wrong size))\n");
            memmove(buffer, buffer + 4, total_bytes-4);
            total_bytes -= (4);
            continue;
        }
        while(total_bytes<TM_HEADER_SIZE + header.cargo_length + 2){
            int bytes_read = read(serial_fd, buffer + total_bytes, READ_CHUNK_SIZE);
            if(bytes_read>0){
                total_bytes += bytes_read;
            }
        }
        //-------------------------------------
        pthread_mutex_lock(&debug_mutex);
        fprintf(debug_file, "Receiving complete message: ");
        for (int i = 0; i < TM_HEADER_SIZE + header.cargo_length + 2; i++) {
            fprintf(debug_file," %02X", buffer[i]);
        }
        fprintf(debug_file,"\n");
        fflush(debug_file);
        pthread_mutex_unlock(&debug_mutex);
        //-------------------------------------

        // Calcola e verifica il CRC
        uint16_t calculated_crc = calculate_crc(buffer, TM_HEADER_SIZE + header.cargo_length);

        uint16_t received_crc;
        memcpy(&received_crc, buffer + TM_HEADER_SIZE + header.cargo_length, 2);


        if (calculated_crc == received_crc) {
            if(header.packet_type == PACKET_TYPE_TM_ACK){
                response=1;
                //fprintf(log_file, "(ACK command #%d received)\n", header.packet_counter);
                memmove(buffer, buffer + 4, total_bytes-4);
                total_bytes -= (4);
                continue;
            }
            else if(header.packet_type == PACKET_TYPE_TM_NAK){
                response=2;
                //fprintf(log_file, "(NAK command #%d received)\n", header.packet_counter);
                memmove(buffer, buffer + 4, total_bytes-4);
                total_bytes -= (4);
                continue;
            }
            pthread_mutex_lock(&log_mutex);
            fprintf(log_file, "(Received TM package #%d) cargo={", header.packet_counter);
            fwrite(buffer + TM_HEADER_SIZE, 1, header.cargo_length, log_file);  // Scrive il cargo nel file
            time_t seconds = header.coarse_time;
            struct tm *timeinfo = gmtime(&seconds);
            uint32_t fine = (header.fine_time[0] << 16) | (header.fine_time[1] << 8) | header.fine_time[2];

            // Converti la frazione da 24 bit a millisecondi (scalandola correttamente)
            uint32_t milliseconds = (fine * 1000) >> 24;

            //uint32_t milliseconds = fine / 1000;

            /*fprintf(log_file, "} (Processor Time: %02d:%02d:%02d.%03d UTC)\n", //for date %04d-%02d-%02d, milliseconds .%03d
                //timeinfo->tm_year + 1900, timeinfo->tm_mon + 1, timeinfo->tm_mday,
                timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec, milliseconds);*/  // Nuova riga per separare i pacchetti
            fprintf(log_file, "} (Processor Time: %02d:%02d:%02d.%03d UTC)\n", 
                timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec, milliseconds);
        } else {
            fprintf(log_file, "(Corrupted TM package)\n");
        }
        fflush(log_file);
        pthread_mutex_unlock(&log_mutex);
        memmove(buffer, buffer + 4, total_bytes-4);
        total_bytes -= (4);
    }
    return NULL;
}


void* sender_thread(void* arg) {
    bool sync = false;
    char command[MAX_COMMAND_LEN]; // Buffer per raccogliere l'input
    char history[HISTORY_SIZE][MAX_COMMAND_LEN];
    uint16_t packet_counter = 0;
    int pointer=0;

    draw_interface();

    while (running) {
        move_cursor(PROMPT_ROW, 18); // Posiziona il cursore dopo "Command to send? "
        printf("\033[K\n\033[K"); // Pulisce la riga precedente
        move_cursor(PROMPT_ROW, 18);
        fflush(stdout);
        
        // Legge l'input dell'utente
        if (fgets(command, sizeof(command), stdin) != NULL) {
            // Rimuove il carattere newline
            command[MAX_COMMAND_LEN-1] = '\0';
            command[strcspn(command, "\n")] = '\0';
            process_command(command, history, &packet_counter, &pointer, &sync);  
            if(sync){
                process_command(command, history, &packet_counter, &pointer, &sync);
                sync = false;
            }
        }
    }
    return NULL;
}


int main() {
    //-----------------------------------------------------------
    // Parte di configurazione
    //-----------------------------------------------------------

    // Configura il segnale
    signal(SIGINT, handle_signal);

    // Variabile di configurazione
    struct termios tty;

    // Prova ad aprire la porta seriale
    serial_fd = open(SERIAL_PORT, O_RDWR | O_NOCTTY | O_NDELAY);
    if (serial_fd == -1) {
        printf("Unable to open serial port %s\n", SERIAL_PORT);
        return 1;
    }

    // Rimuove la modalità non bloccante
    fcntl(serial_fd, F_SETFL, 0);

    // Ottiene la configurazione corrente della porta
    memset(&tty, 0, sizeof(tty));
    if (tcgetattr(serial_fd, &tty) != 0) {
        printf("Unable to read current port configuration\n");
        close(serial_fd);
        return 1;
    }

    // Configurazione del baud rate
    cfsetispeed(&tty, B19200); // Velocità in input
    cfsetospeed(&tty, B19200); // Velocità in output

    // Configurazione della lunghezza della parola, parità e bit di stop
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8 bit
    tty.c_cflag |= PARENB;                      // Abilita la parità
    tty.c_cflag |= PARODD;                      // Parità dispari
    tty.c_cflag &= ~CSTOPB;                     // 1 bit di stop
    tty.c_cflag &= ~CRTSCTS;                    // Disabilita il controllo hardware

    tty.c_iflag &= ~(IGNCR | ICRNL | INLCR); // Evita la conversione di CR a NL
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Disabilita XON/XOFF
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_oflag &= ~(ONLCR | OCRNL);  // Disabilita la traduzione di newline in carriage return + newline
    tty.c_lflag = 0;        // Modalità grezza
    tty.c_cc[VMIN]  = 1;    // Almeno 1 byte necessario

    // Imposta la nuova configurazione
    if (tcsetattr(serial_fd, TCSANOW, &tty) != 0) {
        printf("Unable to set new port configuration\n");
        close(serial_fd);
        return 1;
    }

    // Apre il file di log
    log_file = fopen(LOG_FILE, "a");
    if (!log_file) {
        printf("Unable to open log file %s\n", LOG_FILE);
        close(serial_fd);
        return 1;
    }

    debug_file = fopen(DEBUG_FILE, "a");
    if (!debug_file) {
        printf("Unable to open log file %s\n", DEBUG_FILE);
        close(serial_fd);
        return 1;
    }

    //-----------------------------------------------------------
    // Creazione dei thread
    //-----------------------------------------------------------
    pthread_t receiver, sender;

    // Crea il thread per la ricezione
    if (pthread_create(&receiver, NULL, receiver_thread, NULL) != 0) {
        printf("Failed to create receiver thread\n");
        fclose(log_file);
        close(serial_fd);
        return 1;
    }

    // Crea il thread per la trasmissione
    if (pthread_create(&sender, NULL, sender_thread, NULL) != 0) {
        printf("Failed to create sender thread\n");
        fclose(log_file);
        close(serial_fd);
        return 1;
    }

    // Attende che i thread terminino
    pthread_join(receiver, NULL);
    pthread_join(sender, NULL);

    return 0;
}
