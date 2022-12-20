/*
    void sd_card_make_file(...) create a file at the specified path and writes to the mountpoint specified
    TODO: take in data to write onto a file,, make_file top, read/write_file below
    TODO: implement more detailed checks and configurations

    This function takes in:

    - bool ready: parity bool to ensure call was intentional and proper checks before calling have taken place
    - char *filename: name of the filename you want to create
    - esp_err_t ret: return type that contains error codes, otherwise null
    - sdmmc_card_ *card: sd card object for utilizing (SDMMC)
    - esp_vfs_fat_sdmmc_mount_config_t *mount_config: fat object for filesystem config object
    - const char mount_point[]: specification of path for mounting sdcard (relative root)
    - sdmmc_host_t *host: host object that provides api functionality such as being able to utilize card object properly 
                        utilizes SDMMC peripheral, with 4-bit mode enabled, and max frequency set to 20MHz
*/
// This example uses SDMMC peripheral to communicate with SD card.
#include <errno.h>
#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"
#include "sd_card_headers/sd_card_init.h"
//#include <cJSON.h>
//#include "../../components/json-c/json-c/json_c_version.h"

// static const char *TAG = "make_file";
// #define MOUNT_POINT "/sd_card_v_1"
struct data_packet {
    uint32_t temperature;
    uint32_t humidity;
    uint64_t id;
};
//assumes that the values passed in are correct.
//in the case writing data is size 0, return

esp_err_t sd_card_write_file(bool ready, char *filename, uint8_t *incoming_data, size_t bytes_to_write, const char *TAG, esp_err_t ret,sdmmc_card_t *card, const char *mount_point)
{
    //check to see that the card has been configured and initialize
    if(!ready){
        ESP_LOGE(TAG, "Setup and Configure SD_Card before Creating File(%s)", filename);
    }

    // Use POSIX and C standard library functions to work with files:
    // First create a file.

    //const char *curr_file = "/sdcard/testc.txt";
    char *curr_file = malloc(sizeof(char) *(strlen(mount_point)+strlen(filename) + 1));
    if(curr_file == NULL){
        ret = ESP_FAIL;
        ESP_LOGE(TAG,"Dynamic Mem Allocation Failed for File:%s",filename);
        return ret;
    }
    strlcpy(curr_file, mount_point, strlen(mount_point) + 1);
    //ESP_LOGI(TAG, "curr_file is currently %s", curr_file);
    
    size_t check_sum_size = strlcat(curr_file, filename, strlen(mount_point)+strlen(filename)+1);
    if(check_sum_size < strlen(filename)){
        ESP_LOGE(TAG, "Something went wrong in copying path, Please Check Security/Integrity");
    }

    ESP_LOGI(TAG, "The file will be written to this path: %s", curr_file);
    ESP_LOGI(TAG, "Checking if file %s exists", curr_file);

    // Check if destination file exists before writing
    struct stat st;
    //FILE *f = fopen(curr_file, "w+");
    FILE *f;
    if (stat(curr_file, &st) != 0) {
        //open it 
        ESP_LOGI(TAG, "Opening file %s", curr_file);
        f = fopen(curr_file, "a+");
    }else {
        //create it
        ESP_LOGI(TAG, "Creating new file %s", curr_file);
        f = fopen(curr_file, "w+");
    }
    if (f == NULL) {
        ret = ESP_FAIL;
        ESP_LOGE(TAG, "Failed to open file %s for writing", curr_file);
        free(curr_file);
        return ret;
    }

    //iterate through the contents inside of the data with up to the specified size only or size of file
    //fprintf(f, "Hello %s!\n", card->cid.name);
    //initially thought using stdio was the way to go
    if(sizeof(*incoming_data) <= 0){
        ESP_LOGE(TAG, "The incoming data is empty");
    }

    //using the fat file writing for defualt,, couldn't find proper writing sectors using sdmmc library
    //in specific writing properly
    //ESP_LOGI(TAG, "These are the contents of data (%d)", incoming_data);
    ESP_LOGI(TAG, "These are the cbytes to write  (%d)", bytes_to_write);
    size_t ret_write_val = fwrite(&incoming_data, sizeof(uint8_t), bytes_to_write, f);

    if(ret_write_val < bytes_to_write){
        ESP_LOGE(TAG, "Error in writng data to %s file", curr_file);
    }

    ESP_LOGI(TAG, "Writing File Contents Successful");
    
    char read_hex[sizeof(uint8_t) * bytes_to_write];
    for(int index = 0; index < bytes_to_write; index++){
        sprintf(read_hex+index, "%x", incoming_data[index]);
    }

    ESP_LOGI(TAG, "File should have been added with 0x:%s", read_hex);

    if(fclose(f) != 0){
        ret = ESP_ERR_INVALID_STATE;
        ESP_LOGE(TAG, "For system %s, there was an error in closing the file with error: %s", TAG, strerror(errno));
        fprintf(stderr, "There was an Error in closing the file: %s", strerror(errno));
        free(curr_file);
        return ret;
    }
    ESP_LOGI(TAG, "File %s Successfully Written to Card %s", curr_file, card->cid.name);
    ret = ESP_OK;
    free(curr_file);
    return ret;
}

//sends as a stream or array pointer
esp_err_t sd_card_read_file(bool ready, char *filename, uint8_t *data_array, size_t bytes_to_read, const char *TAG, esp_err_t ret,sdmmc_card_t *card, const char *mount_point){
    //check to see that the card has been configured and initialize
    //check to see that the card has been configured and initialize
    if(!ready){
        ESP_LOGE(TAG, "Setup and Configure SD_Card before Creating File(%s)", filename);
    }

    // Use POSIX and C standard library functions to work with files:
    // First create a file.
    //const char *curr_file = mount_point;
    //const char *curr_file = "/sdcard/testc.txt";

    char *curr_file = malloc(sizeof(char) *(strlen(mount_point)+strlen(filename) + 1));
    if(curr_file == NULL){
        ret = ESP_FAIL;
        ESP_LOGE(TAG,"Dynamic Mem Allocation Failed for File:%s",filename);
        return ret;
    }
    strlcpy(curr_file, mount_point, strlen(mount_point) + 1);
    //ESP_LOGI(TAG, "curr_file is currently %s", curr_file);
    
    size_t check_sum_size = strlcat(curr_file, filename, strlen(mount_point)+strlen(filename)+1);
    if(check_sum_size < strlen(filename)){
        ret = ESP_FAIL;
        ESP_LOGE(TAG, "Something went wrong in copying path, Please Check Security/Integrity");
        free(curr_file);
        return ret;
    }

    ESP_LOGI(TAG, "The file will be read from this path: %s", curr_file);
    ESP_LOGI(TAG, "Checking if file %s exists", curr_file);

    // Open renamed file for reading
    ESP_LOGI(TAG, "Reading file %s", curr_file);
    //struct stat st;
    FILE *f = fopen(curr_file, "r");
    if (f == NULL) {
        ret = ESP_FAIL;
        ESP_LOGE(TAG, "Failed to open file for reading");
        free(curr_file);
        return ret;
    }
    
    uint8_t *bytes_read = malloc(bytes_to_read*sizeof(uint8_t));
    if(bytes_read == NULL){
        ret = ESP_FAIL;
        ESP_LOGE(TAG, "Malloc Failed for reading file");
        free(curr_file);
        //free(bytes_read);
        return ret;
    }

    fread(&bytes_read,sizeof(uint8_t),bytes_to_read,f);
    ESP_LOGI(TAG,"Current first byte looks like: %d", *bytes_read);



    char read_hex[sizeof(uint8_t) * bytes_to_read];
    for(int index = 0; index < bytes_to_read; index++){
        sprintf(read_hex+index, "%x", bytes_read[index]);
        ESP_LOGI(TAG,"Current byte looks like: %x", read_hex[index]);
    }

    //printf("Version: %s\n", json_c_version());
    //printf("Version Number: %d\n", json_c_version_num());
    //free(bytes_read);
    
    if(fclose(f) != 0){
        ret = ESP_ERR_INVALID_STATE;
        free(curr_file);
        //free(bytes_read);
        ESP_LOGE(TAG, "For system %s, there was an error in closing the file with error: %s", TAG, strerror(errno));
        fprintf(stderr, "There was an Error in closing the file: %s", strerror(errno));
        return ret;
    }
    ESP_LOGI(TAG, "File %s Successfully Read from Card %s", curr_file, card->cid.name);
    
    ret = ESP_OK;

    // Strip newline
    // char *pos = strchr(byte_read, '\n');
    // if (pos) {
    //     *pos = '\0';
    // }

    ESP_LOGI(TAG, "Read from file: '%s'", read_hex);
    free(curr_file);
    //free(bytes_read);
    return ret;
}