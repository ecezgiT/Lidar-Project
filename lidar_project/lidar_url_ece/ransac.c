#include "lidar.h"

typedef struct {
    double t;
    int idx;
} Projection;//pp noktasının doğru üzerindeki izdüşümü

static int compare_projection(const void* a, const void* b) {
    const Projection* pa = (const Projection*)a;
    const Projection* pb = (const Projection*)b;
    if (pa->t < pb->t) return -1;
    if (pa->t > pb->t) return 1;
    return 0;
}

int line_ransac(Point* points, int point_count, Line* out_lines, int max_lines) {

    const int MAX_ITER = 2000;
    const double DIST_THRESH = 0.12;
    const double GAP_THRESH = 0.058;
    const int MIN_INLIERS = 8;
    const int MAX_USE_PER_POINT = 2;

    int* used_count = (int*)calloc(point_count, sizeof(int));
    if (!used_count) return 0;

    Projection* proj = (Projection*)malloc(sizeof(Projection) * point_count);
    if (!proj) { free(used_count); return 0; }

    srand((unsigned int)time(NULL));

    int found_lines = 0;

    while (found_lines < max_lines) {
        int best_inlier_count = 0;
        Point best_pstart = { 0,0 }, best_pend = { 0,0 };
        int best_inlier_indices_count = 0;
        int* best_inlier_indices = NULL;

        int remaining = 0;
        for (int i = 0; i < point_count; i++) {
            if (used_count[i] < MAX_USE_PER_POINT) remaining++;
        }
        if (remaining < MIN_INLIERS) break;

        for (int it = 0; it < MAX_ITER; it++) {
            int a = -1, b = -1;
            for (int tries = 0; tries < 50; tries++) {
                int i = rand() % point_count;
                if (used_count[i] >= MAX_USE_PER_POINT) continue;
                a = i; break;
            }
            for (int tries = 0; tries < 100; tries++) {
                int j = rand() % point_count;
                if (j == a || used_count[j] >= MAX_USE_PER_POINT) continue;
                b = j; break;
            }
            if (a < 0 || b < 0) continue;

            Point p1 = points[a];
            Point p2 = points[b];
            double dx = p2.x - p1.x;
            double dy = p2.y - p1.y;
            double norm = sqrt(dx * dx + dy * dy);
            if (norm < 1e-6) continue;//sıfır kontrolü

            Line cand = make_line(p1, p2);//aday doğru

            int proj_count = 0;
            for (int i = 0; i < point_count; i++) {
                if (used_count[i] >= MAX_USE_PER_POINT) continue;
                Point pp = points[i];
                double dist = point_line_distance(cand, pp);
                if (dist <= DIST_THRESH) {
                    double t = ((pp.x - p1.x) * dx + (pp.y - p1.y) * dy) / (norm);
                    proj[proj_count].t = t;
                    proj[proj_count].idx = i;
                    proj_count++;
                }//pp noktalarının aday doğruya uzaklıklarına bakıp threshold değeri altındaysa inlier say project et
            }

            if (proj_count < MIN_INLIERS) continue;

            qsort(proj, proj_count, sizeof(Projection), compare_projection);

            int best_block_len = 0;
            int best_block_start = 0;
            int cur_start = 0;
            int cur_len = 1;
            for (int k = 1; k < proj_count; k++) {
                double gap = proj[k].t - proj[k - 1].t;
                if (gap <= GAP_THRESH) {
                    cur_len++;
                }
                else {
                    if (cur_len > best_block_len) {
                        best_block_len = cur_len;
                        best_block_start = cur_start;
                    }
                    cur_start = k;//yeni blok
                    cur_len = 1;
                }
            }
            if (cur_len > best_block_len) {
                best_block_len = cur_len;
                best_block_start = cur_start;
            }

            if (best_block_len >= MIN_INLIERS) {
                int idx_first = proj[best_block_start].idx;
                int idx_last = proj[best_block_start + best_block_len - 1].idx;
                Point s = points[idx_first];
                Point e = points[idx_last];

                if (best_block_len > best_inlier_count) {
                    if (best_inlier_indices) {
                        free(best_inlier_indices);
                        best_inlier_indices = NULL;
                    }
                    best_inlier_indices_count = best_block_len;
                    best_inlier_indices = (int*)malloc(sizeof(int) * best_inlier_indices_count);
                    if (!best_inlier_indices) { best_inlier_indices_count = 0; continue; }

                    for (int kk = 0; kk < best_block_len; kk++) {
                        best_inlier_indices[kk] = proj[best_block_start + kk].idx;
                    }
                    best_inlier_count = best_block_len;
                    best_pstart = s;
                    best_pend = e;
                }
            }
        }

        if (best_inlier_count < MIN_INLIERS) {
            if (best_inlier_indices) free(best_inlier_indices);
            break;
        }

        Line out;//çıktı doğrusu
        out.A = best_pend.y - best_pstart.y;
        out.B = best_pstart.x - best_pend.x;
        out.C = best_pend.x * best_pstart.y - best_pstart.x * best_pend.y;
        out.inlier_count = best_inlier_count;
        out.p_start = best_pstart;
        out.p_end = best_pend;

        out.inlier_indices = (int*)malloc(sizeof(int) * best_inlier_indices_count);
        if (out.inlier_indices) {
            for (int kk = 0; kk < best_inlier_indices_count; kk++) out.inlier_indices[kk] = best_inlier_indices[kk];
        }

        out_lines[found_lines] = out;
        found_lines++;

        for (int ii = 0; ii < best_inlier_indices_count; ii++) {
            int idx = best_inlier_indices[ii];
            if (idx >= 0 && idx < point_count) {
                used_count[idx]++;
            }
        }

        if (best_inlier_indices) { free(best_inlier_indices); best_inlier_indices = NULL; }
    }

    free(proj);
    free(used_count);
    return found_lines;
}
