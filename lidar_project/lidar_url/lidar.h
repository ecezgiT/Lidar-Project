#ifndef LIDAR_H
#define LIDAR_H

#define _CRT_SECURE_NO_WARNINGS

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <ctype.h>
#include <time.h>
#include <stdint.h>
#include <stdbool.h>

#include <allegro5/allegro.h>
#include <allegro5/allegro_primitives.h>
#include <allegro5/allegro_font.h>
#include <allegro5/allegro_ttf.h>
#include <allegro5/allegro_color.h>
#include <curl/curl.h>

#define MAX_RANGES 10000//dosyadaki max range sayisi
#define BUFFER_SIZE 600000

//ekran boyutları
extern const int SCREEN_W;
extern const int SCREEN_H;
extern const double SCALE;


typedef struct {
    double angle_min;
    double angle_max;
    double angle_increment;
    double range_min;
    double range_max;
    double ranges[MAX_RANGES];
    int original_indices[MAX_RANGES];//filtreleme sırasında index kayması olmasın diye orjinal indexler
    int range_count;
} LidarHeader;

typedef struct {
    double x;
    double y;
} Point;

typedef struct {
    double A, B, C;
    int inlier_count;
    Point p_start;
    Point p_end;
    int* inlier_indices;
} Line;

typedef struct {
    Point position;
    double distance;
    double angle;
    int line1_idx;
    int line2_idx;
    int found;
} IntersectionInfo;

size_t WriteMemoryCallback(void* contents, size_t size, size_t nmemb, void* userp);
int parse_toml_file(const char* url, LidarHeader* header);


int convert_to_cartesian(const LidarHeader* header, Point* points, int max_points);

int line_ransac(Point* points, int point_count, Line* out_lines, int max_lines);

Line make_line(Point p1, Point p2);
double point_line_distance(Line line, Point p);
int point_intersection(Line l1, Line l2, Point* intersection);
double angle_between_lines(Line l1, Line l2);
double distance_to_robot(Point point);

Point world_to_screen(Point world_p);
void draw_lidar_data(Point* points, int point_count, Line* lines, int line_count, IntersectionInfo best_inter, ALLEGRO_FONT* font);

IntersectionInfo find_intersection(Line* lines, int line_count);

#endif 
