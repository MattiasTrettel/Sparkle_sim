#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#define DEBUG_FILE "Sparkle_sim_debug.txt"

int main() {
    FILE *file;
    char buffer[1024];

    // Apri il file in modalità lettura
    file = fopen(DEBUG_FILE, "r");
    if (file == NULL) {
        perror("Errore nell'apertura del file");
        return EXIT_FAILURE;
    }

    // Spostati alla fine del file
    fseek(file, 0, SEEK_END);

    while (1) {
        // Leggi una riga alla volta
        if (fgets(buffer, sizeof(buffer), file) != NULL) {
            printf("%s", buffer);
            fflush(stdout);  // Forza la stampa immediata
        } else {
            usleep(100000);  // Aspetta 100ms prima di controllare di nuovo
            clearerr(file);  // Resetta lo stato EOF nel caso il file venga aggiornato
        }
    }

    fclose(file);
    return 0;
}
