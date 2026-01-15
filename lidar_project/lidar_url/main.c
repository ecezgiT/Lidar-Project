#include "lidar.h"

int main() {
    ALLEGRO_DISPLAY* display = NULL;
    ALLEGRO_EVENT_QUEUE* event_queue = NULL;
    ALLEGRO_FONT* font = NULL;

    if (!al_init()) {
        fprintf(stderr, "Allegro başlatılamadı.\n");
        return -1;
    }
    if (!al_init_primitives_addon()) {
        fprintf(stderr, "Allegro primitives eklentisi baslatilamadi.\n");
        return -1;
    }
    if (!al_install_keyboard()) {
        fprintf(stderr, "Allegro klavye baslatilamadi.\n");
        return -1;
    }

    al_init_font_addon();
    al_init_ttf_addon();

    display = al_create_display(SCREEN_W, SCREEN_H);
    if (!display) {
        fprintf(stderr, "Allegro ekranı olusturulamadi.\n");
        return -1;
    }

    font = al_create_builtin_font();
    if (!font) {
        fprintf(stderr, "Dahili font olusturulamadi.\n");
        al_destroy_display(display);
        return -1;
    }

    event_queue = al_create_event_queue();
    if (!event_queue) {
        fprintf(stderr, "Allegro event queue olusturulamadi.\n");
        al_destroy_font(font);
        al_destroy_display(display);
        return -1;
    }

    al_register_event_source(event_queue, al_get_display_event_source(display));
    al_register_event_source(event_queue, al_get_keyboard_event_source());

    al_set_window_title(display, "LIDAR RANSAC grafik");

    LidarHeader header;
    if (!parse_toml_file("http://abilgisayar.kocaeli.edu.tr/lidar1.toml", &header)) {
        printf("Hata: lidar1.toml indirilemedi veya okunamadi.\n");
        al_rest(2.0);
        al_destroy_font(font);
        al_destroy_display(display);
        al_destroy_event_queue(event_queue);
        return 1;
    }

    Point* points = (Point*)malloc(sizeof(Point) * MAX_RANGES);
    int point_count = convert_to_cartesian(&header, points, MAX_RANGES);
    printf("Filtrelenmis nokta sayisi: %d\n", point_count);

    if (point_count < 8) {
        printf("Hata: RANSAC icin yeterli nokta (en az 8) bulunamadi.\n");
        al_rest(2.0);
        free(points);
        al_destroy_font(font);
        al_destroy_display(display);
        al_destroy_event_queue(event_queue);
        return 1;
    }

    Line* lines = (Line*)malloc(sizeof(Line) * 100);
    if (!lines) {
        fprintf(stderr, "Bellek hatası.\n");
        free(points);
        al_destroy_font(font);
        al_destroy_display(display);
        al_destroy_event_queue(event_queue);
        return 1;
    }

    for (int i = 0; i < 100; i++) { lines[i].inlier_indices = NULL; lines[i].inlier_count = 0; }

    int line_count = line_ransac(points, point_count, lines, 100);
    printf("\\nBulunan dogrular: %d\\n", line_count);

    if (line_count < 2) {
        printf("Uyari: Kesisim bulmak için yeterli dogru (en az 2) tespit edilemedi.\\n");
    }

    for (int i = 0; i < line_count; i++) {
        printf("%d. dogru: %d nokta iceriyor.\\n", i + 1, lines[i].inlier_count);
    }

    IntersectionInfo best = find_intersection(lines, line_count);

    if (best.found) {
        printf("\\nKesisim noktasi bulundu\\n");
        printf("Dogru %d ve Dogru %d\\n", best.line1_idx + 1, best.line2_idx + 1);
        printf("  Kesisim Noktası: (%.3f, %.3f)\\n", best.position.x, best.position.y);
        printf("  Robota Uzaklik: %.3f m\\n", best.distance);
        printf("  Aci: %.2f derece\\n", best.angle);
    }
    else {
        printf("\\nRobota yakin, 60 derece ustu kesisen bir nokta bulunamadi.\\n");
    }

    draw_lidar_data(points, point_count, lines, line_count, best, font);
    al_flip_display();

    printf("\\nGrafik penceresi acildi. Kapatmak icin pencereyi kapatin veya bir tusa basin...\\n");

    bool running = true;
    while (running) {
        ALLEGRO_EVENT ev;
        al_wait_for_event(event_queue, &ev);
        if (ev.type == ALLEGRO_EVENT_DISPLAY_CLOSE) {
            running = false;
        }
        else if (ev.type == ALLEGRO_EVENT_KEY_DOWN) {
            running = false;
        }
    }

    for (int i = 0; i < line_count; i++) {
        if (lines[i].inlier_indices) free(lines[i].inlier_indices);
    }

    free(lines);
    free(points);

    al_destroy_font(font);
    al_destroy_display(display);
    al_destroy_event_queue(event_queue);

    al_shutdown_ttf_addon();
    al_shutdown_font_addon();
    al_shutdown_primitives_addon();
    al_uninstall_keyboard();

    printf("\\nProgram sonlandi.\\n");
    return 0;
}
