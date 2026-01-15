#include "lidar.h"
//ekran boyutları
const int SCREEN_W = 1000;
const int SCREEN_H = 800;
const double SCALE = 125.0;

int convert_to_cartesian(const LidarHeader* header, Point* points, int max_points) {
    int count = 0;
    for (int i = 0; i < header->range_count && count < max_points; i++) {
        double r = header->ranges[i];
        int original_index = header->original_indices[i];
        double th = header->angle_min + original_index * header->angle_increment;
        double x = r * cos(th);
        double y = r * sin(th);
        points[count].x = x;
        points[count].y = y;
        count++;
    }
    return count;
}

Line make_line(Point p1, Point p2) {
    Line line;
    line.A = p2.y - p1.y;
    line.B = p1.x - p2.x;
    line.C = p2.x * p1.y - p1.x * p2.y;
    line.inlier_count = 0;
    line.inlier_indices = NULL;
    line.p_start = p1;
    line.p_end = p2;
    return line;
}

double point_line_distance(Line line, Point p) {
    double den = sqrt(line.A * line.A + line.B * line.B);
    if (den < 1e-12) return 1e12;
    return fabs(line.A * p.x + line.B * p.y + line.C) / den;
}

int point_intersection(Line l1, Line l2, Point* intersection) {
    double det = l1.A * l2.B - l2.A * l1.B;
    if (fabs(det) < 1e-10) {
        return 0;
    }
    intersection->x = (l2.B * (-l1.C) - l1.B * (-l2.C)) / det;
    intersection->y = (l1.A * (-l2.C) - l2.A * (-l1.C)) / det;
    return 1;
}

double angle_between_lines(Line l1, Line l2) {
    double dot = l1.A * l2.A + l1.B * l2.B;
    double norm1 = sqrt(l1.A * l1.A + l1.B * l1.B);
    double norm2 = sqrt(l2.A * l2.A + l2.B * l2.B);
    if (norm1 == 0 || norm2 == 0) return 0.0;
    double cos_th = dot / (norm1 * norm2);
    if (cos_th > 1.0) cos_th = 1.0;
    if (cos_th < -1.0) cos_th = -1.0;
    double th = acos(cos_th);
    th = th * 180.0 / 3.141592653589793;
    if (th > 90.0) th = 180.0 - th;
    return th;
}

double distance_to_robot(Point point) {
    return sqrt(point.x * point.x + point.y * point.y);
}



