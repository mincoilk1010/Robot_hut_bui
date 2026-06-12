#include "navigation.h"
#include "stdlib.h"


extern uint16_t Lidar_Map[181]; // Mang luu gia tri VL53L0X o cac goc quay
extern int16_t current_angle; // Bien luu tru goc quay cua servo
extern int16_t angle_step; // Buoc nhay cua servo

//Dinh nghia cac nguong khoang cach de quyet dinh huong di chuyen (THRESHOLDS)
#define SAFE_DISTANCE 200 // Khoang cach an toan de di chuyen thang (20cm)
#define EDGE_JUMP_THRESHOLD 150 // Khoang cach de phat hien canh (15cm)


Nav_Direction Scan_and_Decide(void) {
    int left_edge_angle = -1;  // Luu goc tim thay mep trai
    int right_edge_angle = -1; // Luu goc tim thay mep phai

    int step = abs(angle_step); //Chuyen step_angle ve so duong tuyet doi

    //Quet tu giua sang trai
    for (int i = 90; i <= 180-step; i += step) {
        int next_angle = i + step; // Tinh goc tiep theo de quet
        if (Lidar_Map[i] > 0 && Lidar_Map[next_angle] > 0) {
            if (Lidar_Map[next_angle] - Lidar_Map[i] > EDGE_JUMP_THRESHOLD) {
                left_edge_angle = i;
                break;
            }
        }
    }

    //Quet tu giua sang phai
    for (int i = 90; i >= step; i -= step) {
        int prev_angle = i - step; // Tinh goc tiep theo de quet
        if (Lidar_Map[i] > 0 && Lidar_Map[prev_angle] > 0) {
            if (Lidar_Map[prev_angle] - Lidar_Map[i] > EDGE_JUMP_THRESHOLD) {
                right_edge_angle = i;
                break;
            }
        }
    }

    //Ra quyet dinh chon re trai hoac re phai
    if(left_edge_angle != -1 && right_edge_angle != -1) {
        int left_gap = left_edge_angle - 90; // Do rong khoang cach tu giua den mep trai
        int right_gap = 90 - right_edge_angle; // Do rong khoang cach tu giua den mep phai

        if(left_gap <= right_gap) {
            return DIR_LEFT; // Re trai
        } else {
            return DIR_RIGHT; // Re phai
        }
    }
    else if (left_edge_angle != -1) {
        return DIR_LEFT; // Re trai
    }
    else if (right_edge_angle != -1) {
        return DIR_RIGHT; // Re phai
    }
    else {
        return DIR_U_TURN; // Quay dau
    }
}