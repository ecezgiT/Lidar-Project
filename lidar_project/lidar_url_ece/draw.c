#include "lidar.h"


IntersectionInfo find_intersection(Line* lines, int line_count) {
    IntersectionInfo best_intersection;
    best_intersection.found = 0;
    best_intersection.distance = 1e99;
    best_intersection.angle = 0.0;

    for (int i = 0; i < line_count; i++) {
        for (int j = i + 1; j < line_count; j++) {
            double angle = angle_between_lines(lines[i], lines[j]);
            if (angle >= 60.0) {
                Point inter;
                if (point_intersection(lines[i], lines[j], &inter)) {
                    double dist = distance_to_robot(inter);
                    if (best_intersection.found == 0 || dist < best_intersection.distance) {
                        best_intersection.found = 1;
                        best_intersection.distance = dist;
                        best_intersection.position = inter;
                        best_intersection.line1_idx = i;
                        best_intersection.line2_idx = j;
                        best_intersection.angle = angle;
                    }
                }
            }
        }
    }
    return best_intersection;
}

//ekrana dönüştürme
Point world_to_screen(Point world_p) {
    Point screen_p;
    screen_p.x = (world_p.x * SCALE) + (SCREEN_W / 2.0);
    screen_p.y = (-world_p.y * SCALE) + (SCREEN_H / 2.0);
    return screen_p;
}


void draw_lidar_data(Point* points, int point_count, Line* lines, int line_count, IntersectionInfo best_inter, ALLEGRO_FONT* font) {

    al_clear_to_color(al_map_rgb(20, 20, 30));

    Point origin = world_to_screen((Point) { 0.0, 0.0 });
    al_draw_line(0, origin.y, SCREEN_W, origin.y, al_map_rgb(100, 100, 100), 1.0);
    al_draw_line(origin.x, 0, origin.x, SCREEN_H, al_map_rgb(100, 100, 100), 1.0);

    for (int i = 0; i < point_count; i++) {
        Point sp = world_to_screen(points[i]);
        al_draw_filled_circle(sp.x, sp.y, 2.0, al_map_rgb(180, 180, 180));
    }

    for (int i = 0; i < line_count; i++) {
        Point sp1 = world_to_screen(lines[i].p_start);
        Point sp2 = world_to_screen(lines[i].p_end);

        al_draw_line(sp1.x, sp1.y, sp2.x, sp2.y, al_map_rgb(0, 255, 0), 2.5);

        for (int j = 0; j < lines[i].inlier_count; j++) {
            int pt_idx = lines[i].inlier_indices[j];
            if (pt_idx >= 0 && pt_idx < point_count) {
                Point inlier_sp = world_to_screen(points[pt_idx]);
                al_draw_filled_circle(inlier_sp.x, inlier_sp.y, 2.0, al_map_rgb(50, 100, 255));
            }
        }

        Point mid = { (lines[i].p_start.x + lines[i].p_end.x) / 2.0,
                      (lines[i].p_start.y + lines[i].p_end.y) / 2.0 };
        Point mid_sp = world_to_screen(mid);
        char label[16];
        sprintf(label, "d%d", i + 1);
        al_draw_text(font, al_map_rgb(200, 255, 200), mid_sp.x + 5, mid_sp.y - 10, ALLEGRO_ALIGN_LEFT, label);
    }

    Point robot_world_pos = { 0.0, 0.0 };
    Point robot_sp = world_to_screen(robot_world_pos);

    if (best_inter.found) {
        al_draw_filled_circle(robot_sp.x, robot_sp.y, 5, al_map_rgb(0, 150, 255));

        Point inter_sp = world_to_screen(best_inter.position);
        al_draw_filled_circle(inter_sp.x, inter_sp.y, 6, al_map_rgb(255, 0, 0));
        al_draw_circle(inter_sp.x, inter_sp.y, 8, al_map_rgb(255, 255, 255), 1.5);

        al_draw_line(robot_sp.x, robot_sp.y, inter_sp.x, inter_sp.y, al_map_rgb(255, 255, 0), 1.5);

        char dist_text[100];
        sprintf(dist_text, "Kesisim uzakligi: %.2f m", best_inter.distance);
        al_draw_text(font, al_map_rgb(255, 255, 255), 10, 10, ALLEGRO_ALIGN_LEFT, dist_text);

        char angle_text[32];
        sprintf(angle_text, "%.1f°", best_inter.angle);
        al_draw_text(font, al_map_rgb(255, 200, 200), inter_sp.x + 10, inter_sp.y - 10, ALLEGRO_ALIGN_LEFT, angle_text);

    }
    else {
        al_draw_filled_circle(robot_sp.x, robot_sp.y, 5, al_map_rgb(100, 100, 100));
        al_draw_text(font, al_map_rgb(255, 100, 100), 10, 10, ALLEGRO_ALIGN_LEFT, "Gecerli kesisim bulunamadi.");
    }
}
