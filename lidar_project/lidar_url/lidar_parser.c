#include "lidar.h"


size_t WriteMemoryCallback(void* contents, size_t size, size_t nmemb, void* userp) {
    size_t total = size * nmemb;
    strncat((char*)userp, (char*)contents, total);
    return total;
}

int parse_toml_file(const char* url, LidarHeader* header) {
    CURL* curl;
    CURLcode res;
    char* buffer = (char*)calloc(1, BUFFER_SIZE);//buffer bellek ayırma
    if (!buffer) {
        fprintf(stderr, "Bellek ayrilamadi.\n");
        return 0;
    }
    //libcurlün başlatılması
    curl_global_init(CURL_GLOBAL_DEFAULT);
    curl = curl_easy_init();
    if (!curl) {
        fprintf(stderr, "libcurl baslatilamadi.\n");
        free(buffer);
        return 0;
    }

    curl_easy_setopt(curl, CURLOPT_URL, url);
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteMemoryCallback);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, buffer);
    curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, 10L);

    res = curl_easy_perform(curl);
    if (res != CURLE_OK) {
        fprintf(stderr, "HTTP istegi basarisiz: %s\n", curl_easy_strerror(res));
        curl_easy_cleanup(curl);
        curl_global_cleanup();//libcurl kapatma
        free(buffer);
        return 0;
    }

    curl_easy_cleanup(curl);
    curl_global_cleanup();
    
    //geçici bir dosya oluşturup bufferın içeriğinini oraya yaz
    FILE* f = tmpfile();
    if (!f) {
        fprintf(stderr, "Gecici dosya olusturulamadi.\n");
        free(buffer);
        return 0;
    }
    fputs(buffer, f);
    rewind(f);

    // TOML dosyasını satır satır okuma
    char line[4096];
    header->range_count = 0;
    int items_read = 0;

    char* buf = (char*)malloc(BUFFER_SIZE);//satırları tutmak için tampon
    if (buf == NULL) {
        fprintf(stderr, "TOML okuma tamponu icin bellek ayrilamadi.\n");
        fclose(f);
        free(buffer);
        return 0;
    }

    while (fgets(line, sizeof(line), f)) {
        if (strstr(line, "angle_min")) {
            if (sscanf(line, "angle_min = %lf", &header->angle_min) == 1) items_read++;
        }
        else if (strstr(line, "angle_max")) {
            if (sscanf(line, "angle_max = %lf", &header->angle_max) == 1) items_read++;
        }
        else if (strstr(line, "angle_increment")) {
            if (sscanf(line, "angle_increment = %lf", &header->angle_increment) == 1) items_read++;
        }
        else if (strstr(line, "range_min")) {
            if (sscanf(line, "range_min = %lf", &header->range_min) == 1) items_read++;
        }
        else if (strstr(line, "range_max")) {
            if (sscanf(line, "range_max = %lf", &header->range_max) == 1) items_read++;
        }
        else if (strstr(line, "ranges")) {
            buf[0] = '\0';
            char* p = strchr(line, '[');
            if (p) {
                strncat(buf, p + 1, BUFFER_SIZE - strlen(buf) - 1);
                while (!strchr(buf, ']') && fgets(line, sizeof(line), f)) {
                    strncat(buf, line, BUFFER_SIZE - strlen(buf) - 1);
                }
                char* end = strchr(buf, ']');
                if (end) *end = '\0';
                //, ile tokenize etme
                char* tok = strtok(buf, ",");
                int current_index = 0;
                while (tok && header->range_count < MAX_RANGES) {
                    while (isspace((unsigned char)*tok)) tok++;
                    if (*tok != '\0') {
                        char* endptr = tok + strlen(tok) - 1;
                        while (endptr > tok && isspace((unsigned char)*endptr))
                            *endptr-- = '\0';
                    }

                    if (*tok == '\0') {
                        tok = strtok(NULL, ",");
                        current_index++;
                        continue;
                    }
                    //string double dönüşümü
                    char* endptr_num = NULL;
                    double v = strtod(tok, &endptr_num);

                    if (endptr_num != tok) {
                        if (!isfinite(v) || v == -1.0 || v == 999.0 || v == -999.0) {
                            tok = strtok(NULL, ",");
                            current_index++;
                            continue;
                        }

                        if (items_read >= 5 && v >= header->range_min && v <= header->range_max) {
                            header->ranges[header->range_count] = v;
                            header->original_indices[header->range_count] = current_index;
                            header->range_count++;
                        }
                    }
                    tok = strtok(NULL, ",");
                    current_index++;
                }
            }
        }
    }

    free(buf);
    fclose(f);
    free(buffer);

    if (items_read < 5) {
        fprintf(stderr, "TOML dosyasindaki tum baslik bilgileri okunamadi.\n");
        return 0;
    }
    return 1;
}
