/*
 * gui.cpp — KAYRA ROV Surface Software dashboard
 *
 * Bright beach / sea themed cockpit with central camera viewport & HUD overlays.
 * Uses SDL2 + OpenGL3 backend, Dear ImGui custom drawing.
 *
 * Layout:
 *
 *   ┌── Title Bar ───────────────────────────────────────────────────┐
 *   ├── Left 18% ──┬───── Camera Viewport 52% ──────┬─ Right 30% ──┤
 *   │  Stick L (○)  │  ┌─ Heading Tape ────────────┐ │ [Surge ◠]   │
 *   │  Stick R (○)  │  │                            │ │ [Sway  ◠]   │
 *   │  Buttons      │  │  Attitude   Depth │ │ [Heave ◠]   │
 *   │  Status       │  │  Indicator  Gauge │ │ [Yaw   ◠]   │
 *   │               │  │  MotionBall ──info strip── │ │             │
 *   │               │  └────────────────────────────┘ │             │
 *   └──────────────┴────────────────────────────────┴─────────────┘
 */

#include <SDL2/SDL.h>
#include <SDL2/SDL_opengl.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <vector>
#include <algorithm>
#include <unordered_set>

#include "imgui.h"
#include "imgui_impl_sdl2.h"
#include "imgui_impl_opengl3.h"

#include "gui.h"

/* ================================================================== */
/*  Constants & beach / sea colour palette                             */
/* ================================================================== */

static const char *GLSL_VER = "#version 130";

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ─── Beach / Sea — light airy base ─── */
#define COL_BG           IM_COL32(224, 242, 248, 255)   /* ice-blue bg          */
#define COL_PANEL        IM_COL32(238, 247, 252, 255)   /* near-white blue      */
#define COL_PANEL_DARK   IM_COL32(208, 230, 242, 255)   /* soft instrument face */
#define COL_BORDER       IM_COL32(136, 192, 218, 255)   /* ocean border         */
#define COL_BORDER_LT    IM_COL32(168, 212, 232, 255)   /* lighter border       */
#define COL_GRID         IM_COL32(188, 216, 232, 255)   /* soft grid            */

/* ─── Text (dark on light) ─── */
#define COL_TEXT         IM_COL32( 26,  59,  92, 255)   /* dark navy            */
#define COL_DIM          IM_COL32(104, 144, 168, 255)   /* muted blue-gray      */

/* ─── Beach accents — varied & cheerful ─── */
#define COL_ACCENT       IM_COL32( 52, 152, 219, 255)   /* ocean blue           */
#define COL_CYAN         IM_COL32(  0, 206, 209, 255)   /* dark turquoise       */
#define COL_CYAN_DIM     IM_COL32(  0, 206, 209,  70)   /* teal transparent     */
#define COL_CYAN_GLOW    IM_COL32(  0, 255, 255,  30)   /* #00FFFF glow         */
#define COL_TEAL         IM_COL32(  0, 180, 190, 255)   /* teal                 */
#define COL_GREEN        IM_COL32( 46, 204, 113, 255)   /* seafoam green        */
#define COL_GREEN_DIM    IM_COL32( 46, 204, 113, 100)   /* seafoam glow         */
#define COL_YELLOW       IM_COL32(240, 178,  80, 255)   /* sandy gold           */
#define COL_RED          IM_COL32(255, 107, 107, 255)   /* coral pink           */
#define COL_ORANGE       IM_COL32(255, 140,  66, 255)   /* sunset orange        */
#define COL_PURPLE       IM_COL32(155, 110, 210, 255)   /* lavender             */

/* ─── Mark colours (dark marks on light bg) ─── */
#define COL_MARK         IM_COL32( 26,  59,  92, 255)   /* dark navy mark       */
#define COL_MARK_DIM     IM_COL32( 26,  59,  92, 120)   /* semi-transparent     */
#define COL_MARK_FAINT   IM_COL32( 26,  59,  92,  50)   /* faint mark           */

/* ─── White (still useful for actual white) ─── */
#define COL_WHITE        IM_COL32(255, 255, 255, 255)
#define COL_WHITE_DIM    IM_COL32(255, 255, 255, 180)
#define COL_WHITE_FAINT  IM_COL32(255, 255, 255, 100)

/* ─── Attitude indicator ─── */
#define COL_SKY          IM_COL32(100, 180, 240, 255)   /* bright sky blue      */
#define COL_GROUND       IM_COL32(218, 185, 120, 255)   /* sandy beach          */
#define COL_HORIZON      IM_COL32(255, 255, 255, 230)   /* white horizon        */
#define COL_AIRCRAFT     IM_COL32(255, 140,  42, 255)   /* sunset orange symbol */

/* ─── Camera / HUD ─── */
#define COL_HUD_BG       IM_COL32(255, 255, 255, 170)   /* white semi-transp    */
#define COL_CAM_FRAME    IM_COL32(  0, 206, 209, 140)   /* teal frame           */

/* ─── Gauge ─── */
#define COL_GAUGE_TRACK  IM_COL32(195, 218, 232, 255)   /* light blue track     */

/* ================================================================== */
/*  Static state                                                      */
/* ================================================================== */

static SDL_Window   *s_win     = nullptr;
static SDL_GLContext  s_gl      = nullptr;
static GLuint         s_cam_tex = 0;
static int            s_cam_tw  = 0;
static int            s_cam_th  = 0;

/* ─── 3-D ROV mesh (decimated from STL) ─── */
struct RovTri {
    float n[3];        /* face normal */
    float v[3][3];     /* 3 vertices, each xyz */
};
static RovTri *s_rov_mesh  = nullptr;
static int     s_rov_count = 0;
static float   s_rov_cx, s_rov_cy, s_rov_cz, s_rov_scale;

/* ================================================================== */
/*  Theme                                                             */
/* ================================================================== */

static void apply_theme()
{
    ImGuiStyle &s = ImGui::GetStyle();
    s.WindowRounding   = 0.0f;
    s.ChildRounding    = 6.0f;
    s.FrameRounding    = 4.0f;
    s.GrabRounding     = 4.0f;
    s.WindowPadding    = ImVec2(10, 8);
    s.ItemSpacing      = ImVec2(8, 6);
    s.WindowBorderSize = 0.0f;
    s.ChildBorderSize  = 1.0f;

    ImVec4 *c = s.Colors;
    c[ImGuiCol_WindowBg]       = ImVec4(0.878f, 0.949f, 0.973f, 1.0f);
    c[ImGuiCol_ChildBg]        = ImVec4(0.918f, 0.961f, 0.980f, 1.0f);
    c[ImGuiCol_Border]         = ImVec4(0.533f, 0.753f, 0.855f, 1.0f);
    c[ImGuiCol_Text]           = ImVec4(0.102f, 0.231f, 0.361f, 1.0f);
    c[ImGuiCol_TextDisabled]   = ImVec4(0.408f, 0.565f, 0.659f, 1.0f);
    c[ImGuiCol_TitleBg]        = ImVec4(0.878f, 0.937f, 0.965f, 1.0f);
    c[ImGuiCol_TitleBgActive]  = ImVec4(0.855f, 0.925f, 0.957f, 1.0f);
    c[ImGuiCol_ScrollbarBg]    = ImVec4(0.878f, 0.949f, 0.973f, 1.0f);
    c[ImGuiCol_ScrollbarGrab]  = ImVec4(0.533f, 0.753f, 0.855f, 1.0f);
    c[ImGuiCol_Button]         = ImVec4(0.800f, 0.900f, 0.945f, 1.0f);
    c[ImGuiCol_ButtonHovered]  = ImVec4(0.700f, 0.860f, 0.930f, 1.0f);
    c[ImGuiCol_ButtonActive]   = ImVec4(0.600f, 0.810f, 0.910f, 1.0f);
    c[ImGuiCol_Separator]      = ImVec4(0.533f, 0.753f, 0.855f, 0.5f);
}

/* ================================================================== */
/*  Helpers                                                           */
/* ================================================================== */

static inline float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : v > hi ? hi : v;
}

static ImU32 severity_color(float fraction)
{
    float f = clampf(fraction, 0.0f, 1.0f);
    if (f < 0.45f) return COL_GREEN;
    if (f < 0.75f) return COL_YELLOW;
    return COL_RED;
}

/* ================================================================== */
/*  Binary STL loader with spatial decimation                         */
/* ================================================================== */

#pragma pack(push,1)
struct BinStlTri {
    float normal[3];
    float v1[3], v2[3], v3[3];
    uint16_t attr;
};
#pragma pack(pop)

static void load_rov_stl(const char *path)
{
    /* Free previous mesh if reloading */
    if (s_rov_mesh) { delete[] s_rov_mesh; s_rov_mesh = nullptr; s_rov_count = 0; }

    FILE *f = fopen(path, "rb");
    if (!f) {
        fprintf(stderr, "[gui] STL not found: %s\n", path);
        return;
    }

    fseek(f, 80, SEEK_SET);
    uint32_t tri_count = 0;
    if (fread(&tri_count, 4, 1, f) != 1) { fclose(f); return; }
    if (tri_count == 0 || tri_count > 10000000u) {
        fprintf(stderr, "[gui] STL tri_count suspicious: %u\n", (unsigned)tri_count);
        fclose(f); return;
    }
    printf("[gui] STL raw triangles: %u\n", (unsigned)tri_count);

    BinStlTri *raw = (BinStlTri *)malloc((size_t)tri_count * sizeof(BinStlTri));
    if (!raw) { fclose(f); return; }
    size_t got = fread(raw, sizeof(BinStlTri), tri_count, f);
    fclose(f);
    if (got != tri_count) { free(raw); return; }

    /* Bounding box */
    float lo[3] = { 1e18f, 1e18f, 1e18f };
    float hi[3] = {-1e18f,-1e18f,-1e18f };
    for (uint32_t i = 0; i < tri_count; i++) {
        const float *vs[3] = { raw[i].v1, raw[i].v2, raw[i].v3 };
        for (int j = 0; j < 3; j++)
            for (int k = 0; k < 3; k++) {
                if (vs[j][k] < lo[k]) lo[k] = vs[j][k];
                if (vs[j][k] > hi[k]) hi[k] = vs[j][k];
            }
    }

    s_rov_cx = (lo[0]+hi[0])*0.5f;
    s_rov_cy = (lo[1]+hi[1])*0.5f;
    s_rov_cz = (lo[2]+hi[2])*0.5f;
    float dx = hi[0]-lo[0], dy = hi[1]-lo[1], dz = hi[2]-lo[2];
    s_rov_scale = dx;
    if (dy > s_rov_scale) s_rov_scale = dy;
    if (dz > s_rov_scale) s_rov_scale = dz;
    s_rov_scale *= 0.5f;
    if (s_rov_scale < 1e-6f) s_rov_scale = 1.0f;

    /* Spatial decimation — voxel grid keeps one tri per cell */
    const int GRID = 48;
    float cell[3] = {
        dx / (float)GRID + 1e-9f,
        dy / (float)GRID + 1e-9f,
        dz / (float)GRID + 1e-9f,
    };

    std::unordered_set<int> occ;
    std::vector<RovTri> kept;
    kept.reserve(8000);

    for (uint32_t i = 0; i < tri_count; i++) {
        float cx = (raw[i].v1[0]+raw[i].v2[0]+raw[i].v3[0]) / 3.0f;
        float cy = (raw[i].v1[1]+raw[i].v2[1]+raw[i].v3[1]) / 3.0f;
        float cz = (raw[i].v1[2]+raw[i].v2[2]+raw[i].v3[2]) / 3.0f;

        int gx = (int)((cx - lo[0]) / cell[0]);
        int gy = (int)((cy - lo[1]) / cell[1]);
        int gz = (int)((cz - lo[2]) / cell[2]);
        if (gx >= GRID) gx = GRID-1;
        if (gy >= GRID) gy = GRID-1;
        if (gz >= GRID) gz = GRID-1;

        int key = gz*GRID*GRID + gy*GRID + gx;
        if (occ.count(key)) continue;
        occ.insert(key);

        RovTri t;
        t.n[0] = raw[i].normal[0];
        t.n[1] = raw[i].normal[1];
        t.n[2] = raw[i].normal[2];
        const float *src[3] = { raw[i].v1, raw[i].v2, raw[i].v3 };
        for (int j = 0; j < 3; j++)
            for (int k = 0; k < 3; k++)
                t.v[j][k] = src[j][k];
        kept.push_back(t);
    }
    free(raw);

    s_rov_count = (int)kept.size();
    s_rov_mesh = new RovTri[s_rov_count];
    memcpy(s_rov_mesh, kept.data(), s_rov_count * sizeof(RovTri));

    printf("[gui] STL decimated: %d triangles kept\n", s_rov_count);
}

/* ================================================================== */
/*  Widget: Battery bar                                               */
/* ================================================================== */

static void draw_battery_bar(ImDrawList *dl, ImVec2 pos, float w, float h,
                              float voltage, float current, float percent)
{
    float x = pos.x, y = pos.y;
    float pct = clampf(percent, 0, 100) / 100.0f;

    /* Outer shell */
    float nw = 5, nh = h * 0.40f;
    dl->AddRectFilled(pos, ImVec2(x + w, y + h),
                      IM_COL32(230, 240, 248, 255), 4);
    dl->AddRect(pos, ImVec2(x + w, y + h), COL_BORDER, 4, 0, 1.5f);
    /* Terminal nub */
    dl->AddRectFilled(ImVec2(x + w, y + (h - nh) * 0.5f),
                      ImVec2(x + w + nw, y + (h + nh) * 0.5f),
                      COL_BORDER, 2);

    /* Fill colour */
    ImU32 fc;
    if (percent > 50)      fc = COL_GREEN;
    else if (percent > 20) fc = COL_YELLOW;
    else                   fc = COL_RED;

    float fw = (w - 6) * pct;
    dl->AddRectFilled(ImVec2(x + 3, y + 3), ImVec2(x + 3 + fw, y + h - 3),
                      fc, 2);
    /* Soft glow */
    dl->AddRectFilled(ImVec2(x + 2, y + 2), ImVec2(x + 4 + fw, y + h - 2),
                      (fc & 0x00FFFFFF) | 0x25000000, 3);

    /* Overlay text */
    char tb[48];
    snprintf(tb, sizeof(tb), "%.1fV  %.0f%%", voltage, percent);
    ImVec2 ts = ImGui::CalcTextSize(tb);
    dl->AddText(ImVec2(x + (w - ts.x) * 0.5f, y + (h - ts.y) * 0.5f),
                COL_TEXT, tb);

    /* Current below */
    if (current > 0.01f) {
        char cb[32];
        snprintf(cb, sizeof(cb), "%.1f A", current);
        dl->AddText(ImVec2(x, y + h + 3), COL_DIM, cb);
    }
}

/* ================================================================== */
/*  Widget: 3-D ROV model (projected mesh)                            */
/* ================================================================== */

static void draw_rov_3d(ImDrawList *dl, ImVec2 center, float widget_r,
                         float roll_deg, float pitch_deg, float yaw_deg)
{
    /* Background disc */
    dl->AddCircleFilled(center, widget_r, IM_COL32(218, 238, 248, 235), 48);
    dl->AddCircle(center, widget_r, COL_BORDER, 48, 1.5f);

    if (s_rov_count == 0) {
        /* Fallback: wireframe cube placeholder */
        const char *msg = "No STL";
        ImVec2 ms = ImGui::CalcTextSize(msg);
        dl->AddText(ImVec2(center.x - ms.x*0.5f, center.y - ms.y*0.5f),
                    COL_DIM, msg);
        return;
    }

    dl->PushClipRect(ImVec2(center.x - widget_r, center.y - widget_r),
                     ImVec2(center.x + widget_r, center.y + widget_r), true);

    float scale = widget_r * 0.70f / s_rov_scale;
    float d2r = (float)M_PI / 180.0f;

    /* Rotation matrix: Rz(yaw) · Ry(pitch) · Rx(roll) */
    float cr = cosf(roll_deg*d2r),  sr = sinf(roll_deg*d2r);
    float cp = cosf(pitch_deg*d2r), sp = sinf(pitch_deg*d2r);
    float cy = cosf(yaw_deg*d2r),   sy = sinf(yaw_deg*d2r);

    float m[3][3] = {
        { cy*cp,  cy*sp*sr - sy*cr,  cy*sp*cr + sy*sr },
        { sy*cp,  sy*sp*sr + cy*cr,  sy*sp*cr - cy*sr },
        { -sp,    cp*sr,             cp*cr             },
    };

    /* Light direction (top-left-front) */
    float lx = 0.35f, ly = -0.65f, lz = 0.65f;
    float ll = sqrtf(lx*lx + ly*ly + lz*lz);
    lx /= ll; ly /= ll; lz /= ll;

    struct PTri { ImVec2 v[3]; float depth; ImU32 color; };
    static std::vector<PTri> proj;
    proj.resize(s_rov_count);
    int valid = 0;

    for (int i = 0; i < s_rov_count; i++) {
        const RovTri &tri = s_rov_mesh[i];
        PTri pt;
        float avg_z = 0;

        for (int j = 0; j < 3; j++) {
            float x = tri.v[j][0] - s_rov_cx;
            float y = tri.v[j][1] - s_rov_cy;
            float z = tri.v[j][2] - s_rov_cz;
            float rx = m[0][0]*x + m[0][1]*y + m[0][2]*z;
            float ry = m[1][0]*x + m[1][1]*y + m[1][2]*z;
            float rz = m[2][0]*x + m[2][1]*y + m[2][2]*z;
            pt.v[j] = ImVec2(center.x + rx*scale, center.y - ry*scale);
            avg_z += rz;
        }

        /* Back-face culling */
        float cross = (pt.v[1].x - pt.v[0].x) * (pt.v[2].y - pt.v[0].y)
                    - (pt.v[1].y - pt.v[0].y) * (pt.v[2].x - pt.v[0].x);
        if (cross > 0) continue;

        pt.depth = avg_z;

        /* Flat shading */
        float nx = m[0][0]*tri.n[0] + m[0][1]*tri.n[1] + m[0][2]*tri.n[2];
        float ny = m[1][0]*tri.n[0] + m[1][1]*tri.n[1] + m[1][2]*tri.n[2];
        float nz = m[2][0]*tri.n[0] + m[2][1]*tri.n[1] + m[2][2]*tri.n[2];
        float dot = nx*lx + ny*ly + nz*lz;
        if (dot < 0) dot = -dot * 0.25f;         /* back-light */
        float b = 0.22f + 0.78f * dot;

        /* Beach-ocean colour gradient */
        int rr = (int)(b * 85 + 70);
        int gg = (int)(b * 115 + 110);
        int bb = (int)(b * 75 + 175);
        pt.color = IM_COL32(rr, gg, bb, 255);

        proj[valid++] = pt;
    }

    /* Depth-sort (painter's algorithm) */
    std::sort(proj.begin(), proj.begin() + valid,
              [](const PTri &a, const PTri &b) { return a.depth < b.depth; });

    for (int i = 0; i < valid; i++)
        dl->AddTriangleFilled(proj[i].v[0], proj[i].v[1], proj[i].v[2],
                              proj[i].color);

    /* Faint edge wireframe for definition */
    for (int i = valid - 1; i >= 0 && i >= valid - 120; i--)
        dl->AddTriangle(proj[i].v[0], proj[i].v[1], proj[i].v[2],
                        IM_COL32(26, 59, 92, 25), 0.5f);

    dl->PopClipRect();

    /* Label */
    dl->AddText(ImVec2(center.x - widget_r + 5, center.y - widget_r + 4),
                COL_DIM, "ROV 3D");

    /* Compass letters around edge */
    const char *dirs[] = { "F", "R", "B", "L" };
    float offsets[] = { -(float)M_PI/2, 0, (float)M_PI/2, (float)M_PI };
    for (int i = 0; i < 4; i++) {
        float a = offsets[i] + yaw_deg * d2r;
        float dx = cosf(a) * (widget_r - 10);
        float dy = sinf(a) * (widget_r - 10);
        dl->AddText(ImVec2(center.x + dx - 3, center.y + dy - 6),
                    COL_DIM, dirs[i]);
    }
}

/* ================================================================== */
/*  Camera texture upload                                             */
/* ================================================================== */

static void update_camera_texture(const uint8_t *rgb, int w, int h)
{
    if (!rgb || w <= 0 || h <= 0) return;

    if (s_cam_tex == 0 || s_cam_tw != w || s_cam_th != h) {
        if (s_cam_tex) glDeleteTextures(1, &s_cam_tex);
        glGenTextures(1, &s_cam_tex);
        glBindTexture(GL_TEXTURE_2D, s_cam_tex);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, w, h, 0,
                     GL_RGB, GL_UNSIGNED_BYTE, rgb);
        s_cam_tw = w;
        s_cam_th = h;
    } else {
        glBindTexture(GL_TEXTURE_2D, s_cam_tex);
        glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, w, h,
                        GL_RGB, GL_UNSIGNED_BYTE, rgb);
    }
}

/* ================================================================== */
/*  Widget: Attitude Indicator (Artificial Horizon)                   */
/* ================================================================== */

static void draw_attitude_indicator(ImDrawList *dl, ImVec2 ctr, float radius,
                                     float pitch_norm, float roll_norm)
{
    float r  = radius;
    float cx = ctr.x, cy = ctr.y;

    float pitch_px = clampf(pitch_norm, -1.0f, 1.0f) * r * 0.7f;
    float roll_rad = clampf(roll_norm,  -1.0f, 1.0f) * 35.0f * (float)M_PI / 180.0f;
    float ca = cosf(roll_rad), sa = sinf(roll_rad);

    /* Outer glow */
    dl->AddCircle(ctr, r + 4, COL_CYAN_GLOW, 64, 8.0f);

    /* 1. Fill circle with sky */
    dl->AddCircleFilled(ctr, r, COL_SKY, 64);
    dl->AddCircleFilled(ctr, r * 0.50f, IM_COL32(135, 200, 245, 40), 48);

    /* 2. Compute horizon–circle intersection & fill ground */
    float disc = r * r - pitch_px * pitch_px * ca * ca;

    if (disc >= 0.0f) {
        float sq = sqrtf(disc);
        float t1 = -pitch_px * sa - sq;
        float t2 = -pitch_px * sa + sq;

        ImVec2 P1(cx + t1 * ca, cy + pitch_px + t1 * sa);
        ImVec2 P2(cx + t2 * ca, cy + pitch_px + t2 * sa);

        float a1 = atan2f(P1.y - cy, P1.x - cx);
        float a2 = atan2f(P2.y - cy, P2.x - cx);
        if (a2 <= a1) a2 += 2.0f * (float)M_PI;

        float mid     = (a1 + a2) * 0.5f;
        float test_x  = cx + r * cosf(mid);
        float test_y  = cy + r * sinf(mid);
        float gnd_dot = (test_x - cx) * (-sa) + (test_y - cy - pitch_px) * ca;

        const int N_ARC = 48;
        ImVec2 poly[N_ARC + 4];
        int np = 0;

        if (gnd_dot > 0.0f) {
            poly[np++] = P1;
            for (int i = 1; i < N_ARC; i++) {
                float a = a1 + (a2 - a1) * (float)i / (float)N_ARC;
                poly[np++] = ImVec2(cx + r * cosf(a), cy + r * sinf(a));
            }
            poly[np++] = P2;
        } else {
            float a1_ext = a1 + 2.0f * (float)M_PI;
            poly[np++] = P2;
            for (int i = 1; i < N_ARC; i++) {
                float a = a2 + (a1_ext - a2) * (float)i / (float)N_ARC;
                poly[np++] = ImVec2(cx + r * cosf(a), cy + r * sinf(a));
            }
            poly[np++] = P1;
        }

        if (np >= 3)
            dl->AddConvexPolyFilled(poly, np, COL_GROUND);

        dl->AddLine(P1, P2, IM_COL32(255, 255, 255, 30), 8.0f);
        dl->AddLine(P1, P2, COL_HORIZON, 2.0f);
    } else {
        if (-pitch_px * ca > 0.0f)
            dl->AddCircleFilled(ctr, r, COL_GROUND, 64);
    }

    /* 3. Pitch ladder */
    float spacing = r * 0.16f;
    float upx = sinf(roll_rad), upy = -cosf(roll_rad);

    for (int deg = -3; deg <= 3; deg++) {
        if (deg == 0) continue;
        int ad = deg < 0 ? -deg : deg;
        float half = (ad == 1) ? r * 0.24f
                   : (ad == 2) ? r * 0.17f
                   : r * 0.11f;

        float lcx = cx + (float)deg * spacing * upx;
        float lcy = (cy + pitch_px) + (float)deg * spacing * upy;
        float ex  = half * ca, ey = half * sa;

        ImVec2 la(lcx - ex, lcy - ey);
        ImVec2 lb(lcx + ex, lcy + ey);
        float r2 = (r - 8.0f) * (r - 8.0f);
        float da = (la.x - cx) * (la.x - cx) + (la.y - cy) * (la.y - cy);
        float db = (lb.x - cx) * (lb.x - cx) + (lb.y - cy) * (lb.y - cy);

        if (da < r2 && db < r2) {
            dl->AddLine(la, lb, COL_WHITE_DIM, 1.0f);
            float tick = (deg > 0) ? 4.0f : -4.0f;
            dl->AddLine(la, ImVec2(la.x - tick * upx, la.y - tick * upy),
                        COL_WHITE_DIM, 1.0f);
            dl->AddLine(lb, ImVec2(lb.x - tick * upx, lb.y - tick * upy),
                        COL_WHITE_DIM, 1.0f);
        }
    }

    /* 4. Roll arc + ticks */
    float rticks[] = { -60, -45, -30, -20, -10, 0, 10, 20, 30, 45, 60 };
    for (int i = 0; i < 11; i++) {
        float ta = (-90.0f + rticks[i]) * (float)M_PI / 180.0f;
        bool maj = (rticks[i] == 0.0f ||
                    fabsf(rticks[i]) == 30.0f ||
                    fabsf(rticks[i]) == 60.0f);
        float inner = maj ? r - 14.0f : r - 8.0f;
        dl->AddLine(
            ImVec2(cx + cosf(ta) * inner, cy + sinf(ta) * inner),
            ImVec2(cx + cosf(ta) * (r - 2), cy + sinf(ta) * (r - 2)),
            rticks[i] == 0.0f ? COL_WHITE : COL_MARK_DIM,
            rticks[i] == 0.0f ? 2.0f : 1.0f);
    }

    /* Roll pointer */
    {
        float pa = -(float)M_PI / 2.0f + roll_rad;
        float pr = r - 16.0f;
        ImVec2 tip(cx + cosf(pa) * (pr + 12), cy + sinf(pa) * (pr + 12));
        float perp = pa + (float)M_PI / 2.0f;
        ImVec2 pl(cx + cosf(pa) * pr + cosf(perp) * 5,
                  cy + sinf(pa) * pr + sinf(perp) * 5);
        ImVec2 prr(cx + cosf(pa) * pr - cosf(perp) * 5,
                   cy + sinf(pa) * pr - sinf(perp) * 5);
        dl->AddTriangleFilled(tip, pl, prr, COL_CYAN);
    }

    /* 5. Aircraft symbol (fixed) */
    {
        float ws = r * 0.22f, wt = 3.0f;
        dl->AddCircleFilled(ctr, 4.0f, COL_AIRCRAFT);
        dl->AddCircle(ctr, 4.0f, IM_COL32(180, 100, 20, 170), 0, 1.5f);
        dl->AddLine(ImVec2(cx - 8, cy), ImVec2(cx - ws, cy), COL_AIRCRAFT, wt);
        dl->AddLine(ImVec2(cx - ws, cy), ImVec2(cx - ws, cy + 8), COL_AIRCRAFT, wt);
        dl->AddLine(ImVec2(cx + 8, cy), ImVec2(cx + ws, cy), COL_AIRCRAFT, wt);
        dl->AddLine(ImVec2(cx + ws, cy), ImVec2(cx + ws, cy + 8), COL_AIRCRAFT, wt);
        dl->AddLine(ImVec2(cx, cy + 12), ImVec2(cx, cy + 20), COL_AIRCRAFT, 2.0f);
        dl->AddLine(ImVec2(cx - 6, cy + 20), ImVec2(cx + 6, cy + 20),
                    COL_AIRCRAFT, 2.0f);
    }

    /* 6. Ring */
    dl->AddCircle(ctr, r, COL_BORDER_LT, 64, 2.5f);
    dl->AddCircle(ctr, r + 1, IM_COL32(136, 192, 218, 80), 64, 1.0f);

    /* 7. Digital readouts */
    char pb[16], rb[16];
    snprintf(pb, sizeof(pb), "P %+.0f\xc2\xb0", pitch_norm * 35.0f);
    snprintf(rb, sizeof(rb), "R %+.0f\xc2\xb0", roll_norm * 35.0f);
    dl->AddText(ImVec2(cx - r + 6, cy + r - 18), COL_CYAN, pb);
    ImVec2 rsz = ImGui::CalcTextSize(rb);
    dl->AddText(ImVec2(cx + r - rsz.x - 6, cy + r - 18), COL_CYAN, rb);
}

/* ================================================================== */
/*  Widget: Heading Tape (horizontal compass strip)                   */
/* ================================================================== */

static void draw_heading_tape(ImDrawList *dl, ImVec2 pos, float width,
                               float height, float heading_deg)
{
    float x = pos.x, y = pos.y;
    float center_x = x + width * 0.5f;
    float ppd = width / 120.0f;      /* pixels-per-degree (±60° visible) */

    /* Background strip */
    dl->AddRectFilled(pos, ImVec2(x + width, y + height), COL_HUD_BG, 3.0f);

    dl->PushClipRect(pos, ImVec2(x + width, y + height), true);

    int lo = (int)heading_deg - 65;
    int hi = (int)heading_deg + 65;

    for (int deg = lo; deg <= hi; deg++) {
        int nd = ((deg % 360) + 360) % 360;
        float sx = center_x + ((float)deg - heading_deg) * ppd;

        bool cardinal = (nd % 90 == 0);
        bool major    = (nd % 30 == 0);
        bool minor10  = (nd % 10 == 0);
        bool minor5   = (nd %  5 == 0);

        if (!minor5) continue;

        float th;
        ImU32 tc;
        float tw;

        if (cardinal) {
            th = height * 0.45f; tc = COL_TEXT; tw = 2.0f;
        } else if (major) {
            th = height * 0.30f; tc = COL_DIM;  tw = 1.5f;
        } else if (minor10) {
            th = height * 0.18f; tc = COL_GRID;  tw = 1.0f;
        } else {
            th = height * 0.10f; tc = IM_COL32(160, 195, 215, 255); tw = 0.8f;
        }

        dl->AddLine(ImVec2(sx, y + height),
                    ImVec2(sx, y + height - th), tc, tw);

        if (cardinal || major) {
            const char *lbl;
            char buf[8];
            if      (nd ==   0) lbl = "N";
            else if (nd ==  90) lbl = "E";
            else if (nd == 180) lbl = "S";
            else if (nd == 270) lbl = "W";
            else { snprintf(buf, sizeof(buf), "%d", nd); lbl = buf; }

            ImVec2 ls = ImGui::CalcTextSize(lbl);
            ImU32 lc  = (nd == 0) ? COL_RED : (cardinal ? COL_TEXT : COL_DIM);
            dl->AddText(ImVec2(sx - ls.x * 0.5f, y + 3), lc, lbl);
        }
    }

    dl->PopClipRect();

    /* Border */
    dl->AddRect(pos, ImVec2(x + width, y + height), COL_BORDER, 3.0f, 0, 1.0f);

    /* Centre pointer triangle */
    dl->AddTriangleFilled(
        ImVec2(center_x, y + height - 2),
        ImVec2(center_x - 6, y + height + 6),
        ImVec2(center_x + 6, y + height + 6),
        COL_CYAN);

    /* Digital heading box */
    float disp = fmodf(heading_deg + 360.0f, 360.0f);
    char hb[12];
    snprintf(hb, sizeof(hb), "%03.0f\xc2\xb0", disp);
    ImVec2 hs = ImGui::CalcTextSize(hb);
    float bw = hs.x + 14, bh = hs.y + 6;
    ImVec2 btl(center_x - bw * 0.5f, y + height + 7);
    ImVec2 bbr(center_x + bw * 0.5f, y + height + 7 + bh);
    dl->AddRectFilled(btl, bbr, IM_COL32(255, 255, 255, 230), 3.0f);
    dl->AddRect(btl, bbr, COL_BORDER, 3.0f);
    dl->AddText(ImVec2(center_x - hs.x * 0.5f, btl.y + 3), COL_TEXT, hb);
}

/* ================================================================== */
/*  Widget: Arc Gauge (270° sweep)                                    */
/* ================================================================== */

static void draw_arc_gauge(ImDrawList *dl, ImVec2 ctr, float radius,
                            float value, float min_val, float max_val,
                            const char *label, ImU32 accent_color)
{
    float r  = radius;
    float cx = ctr.x, cy = ctr.y;
    float start_a = 3.0f * (float)M_PI / 4.0f;
    float sweep   = 3.0f * (float)M_PI / 2.0f;

    float norm = clampf((value - min_val) / (max_val - min_val), 0.0f, 1.0f);
    float center_frac = (-min_val) / (max_val - min_val);

    dl->AddCircle(ctr, r + 2, IM_COL32(136, 192, 218, 60), 48, 4.0f);
    dl->AddCircleFilled(ctr, r, IM_COL32(240, 248, 252, 250), 48);

    const int SEGS = 60;
    float arc_r = r * 0.76f;
    float thick = r * 0.13f;
    float inner = arc_r - thick * 0.5f;
    float outer = arc_r + thick * 0.5f;

    /* Track */
    for (int i = 0; i < SEGS; i++) {
        float f0 = (float)i / SEGS, f1 = (float)(i + 1) / SEGS;
        float a0 = start_a + f0 * sweep, a1 = start_a + f1 * sweep;
        ImVec2 p[4] = {
            {cx + cosf(a0) * inner, cy + sinf(a0) * inner},
            {cx + cosf(a0) * outer, cy + sinf(a0) * outer},
            {cx + cosf(a1) * outer, cy + sinf(a1) * outer},
            {cx + cosf(a1) * inner, cy + sinf(a1) * inner},
        };
        dl->AddConvexPolyFilled(p, 4, COL_GAUGE_TRACK);
    }

    /* Active fill */
    float draw_lo = (norm < center_frac) ? norm : center_frac;
    float draw_hi = (norm < center_frac) ? center_frac : norm;

    for (int i = 0; i < SEGS; i++) {
        float f0 = (float)i / SEGS, f1 = (float)(i + 1) / SEGS;
        if (f1 <= draw_lo || f0 >= draw_hi) continue;
        float cf0 = f0 < draw_lo ? draw_lo : f0;
        float cf1 = f1 > draw_hi ? draw_hi : f1;
        float a0  = start_a + cf0 * sweep;
        float a1  = start_a + cf1 * sweep;
        float dist = fabsf(((cf0 + cf1) * 0.5f) - center_frac) * 2.0f;
        ImU32 col  = severity_color(dist);
        ImVec2 p[4] = {
            {cx + cosf(a0) * inner, cy + sinf(a0) * inner},
            {cx + cosf(a0) * outer, cy + sinf(a0) * outer},
            {cx + cosf(a1) * outer, cy + sinf(a1) * outer},
            {cx + cosf(a1) * inner, cy + sinf(a1) * inner},
        };
        dl->AddConvexPolyFilled(p, 4, col);
    }

    /* Glow at active edge */
    if (fabsf(norm - center_frac) > 0.02f) {
        float ga = start_a + draw_hi * sweep;
        ImVec2 gp(cx + cosf(ga) * arc_r, cy + sinf(ga) * arc_r);
        dl->AddCircleFilled(gp, thick * 1.6f,
                            (accent_color & 0x00FFFFFF) | 0x1E000000);
    }

    /* Ticks */
    for (int i = 0; i <= 10; i++) {
        float frac = (float)i / 10.0f;
        float a = start_a + frac * sweep;
        bool maj = (i % 5 == 0);
        float ti = outer + 2.0f, to = ti + (maj ? 10.0f : 5.0f);
        dl->AddLine(
            ImVec2(cx + cosf(a) * ti, cy + sinf(a) * ti),
            ImVec2(cx + cosf(a) * to, cy + sinf(a) * to),
            maj ? COL_DIM : COL_GRID, maj ? 1.5f : 1.0f);
    }

    /* Centre mark triangle */
    {
        float ma = start_a + center_frac * sweep;
        float mr = outer + 2.0f;
        ImVec2 tip(cx + cosf(ma) * (mr - 4), cy + sinf(ma) * (mr - 4));
        float perp = ma + (float)M_PI / 2.0f;
        ImVec2 pl(cx + cosf(ma) * (mr + 6) + cosf(perp) * 3,
                  cy + sinf(ma) * (mr + 6) + sinf(perp) * 3);
        ImVec2 prr(cx + cosf(ma) * (mr + 6) - cosf(perp) * 3,
                   cy + sinf(ma) * (mr + 6) - sinf(perp) * 3);
        dl->AddTriangleFilled(tip, pl, prr, COL_MARK_DIM);
    }

    /* Needle — dark mark on light bg */
    {
        float na = start_a + norm * sweep;
        ImVec2 nb(cx + cosf(na) * (arc_r - thick), cy + sinf(na) * (arc_r - thick));
        ImVec2 nt(cx + cosf(na) * (outer + 4), cy + sinf(na) * (outer + 4));
        dl->AddLine(nb, nt, COL_MARK, 2.0f);
        dl->AddCircleFilled(nt, 3.0f, COL_MARK);
    }

    /* Digital value */
    char vb[16];
    snprintf(vb, sizeof(vb), "%+.0f", value);
    ImVec2 vs = ImGui::CalcTextSize(vb);
    dl->AddText(ImVec2(cx - vs.x * 0.5f, cy - vs.y * 0.5f - 4), COL_TEXT, vb);

    /* Label */
    ImVec2 ls = ImGui::CalcTextSize(label);
    dl->AddText(ImVec2(cx - ls.x * 0.5f, cy + 12), COL_DIM, label);

    dl->AddCircle(ctr, 8.0f, (accent_color & 0x00FFFFFF) | 0x50000000,
                  24, 1.5f);
    dl->AddCircle(ctr, r, COL_BORDER, 48, 1.5f);
}

/* ================================================================== */
/*  Widget: Circular Joystick Stick                                   */
/* ================================================================== */

static void draw_circular_stick(ImDrawList *dl, ImVec2 ctr, float radius,
                                 float ax, float ay, const char *label)
{
    float r = radius, cx = ctr.x, cy = ctr.y;

    dl->AddCircleFilled(ctr, r, COL_PANEL_DARK, 48);
    dl->AddCircle(ctr, r * 0.33f, COL_GRID, 32, 1.0f);
    dl->AddCircle(ctr, r * 0.66f, COL_GRID, 48, 1.0f);
    dl->AddLine(ImVec2(cx, cy - r), ImVec2(cx, cy + r), COL_GRID, 1.0f);
    dl->AddLine(ImVec2(cx - r, cy), ImVec2(cx + r, cy), COL_GRID, 1.0f);

    float nx = clampf(ax, -1, 1), ny = clampf(ay, -1, 1);
    /* Clamp magnitude so the dot stays inside the circle */
    float mag = sqrtf(nx * nx + ny * ny);
    if (mag > 1.0f) { nx /= mag; ny /= mag; mag = 1.0f; }
    float px = cx + nx * (r - 8), py = cy + ny * (r - 8);

    if (mag > 0.02f) dl->AddLine(ctr, ImVec2(px, py), COL_CYAN_DIM, 1.5f);

    dl->AddCircleFilled(ImVec2(px, py), 14, COL_CYAN_GLOW);
    dl->AddCircleFilled(ImVec2(px, py),  8, IM_COL32(0, 255, 255, 50));
    dl->AddCircleFilled(ImVec2(px, py),  5, COL_CYAN);
    dl->AddCircle(ImVec2(px, py), 5, IM_COL32(255, 255, 255, 100), 0, 1);
    dl->AddLine(ImVec2(px - 10, py), ImVec2(px + 10, py), COL_CYAN_DIM, 1);
    dl->AddLine(ImVec2(px, py - 10), ImVec2(px, py + 10), COL_CYAN_DIM, 1);

    dl->AddCircle(ctr, r, COL_BORDER, 48, 1.5f);

    ImVec2 ls = ImGui::CalcTextSize(label);
    dl->AddText(ImVec2(cx - ls.x * 0.5f, cy + r + 5), COL_DIM, label);

    char xb[12], yb[12];
    snprintf(xb, sizeof(xb), "X:%+.2f", ax);
    snprintf(yb, sizeof(yb), "Y:%+.2f", ay);
    ImVec2 ys = ImGui::CalcTextSize(yb);
    dl->AddText(ImVec2(cx - r, cy + r + 18), COL_DIM, xb);
    dl->AddText(ImVec2(cx + r - ys.x, cy + r + 18), COL_DIM, yb);
}

/* ================================================================== */
/*  Widget: Depth / Heave vertical gauge                              */
/* ================================================================== */

static void draw_depth_gauge(ImDrawList *dl, ImVec2 pos, float w, float h,
                              float heave_norm, int16_t heave_raw)
{
    float x = pos.x, y = pos.y;

    /* Background — light semi-transparent */
    dl->AddRectFilled(pos, ImVec2(x + w, y + h), IM_COL32(255, 255, 255, 180), 4);
    dl->AddRect(pos, ImVec2(x + w, y + h), COL_BORDER, 4);

    float cy = y + h * 0.5f;
    dl->AddLine(ImVec2(x + 2, cy), ImVec2(x + w - 2, cy),
                COL_BORDER_LT, 1.5f);

    for (int i = 0; i <= 10; i++) {
        float ty = y + (float)i / 10.0f * h;
        bool maj = (i % 5 == 0);
        dl->AddLine(ImVec2(x, ty), ImVec2(x + (maj ? 8.0f : 4.0f), ty),
                    maj ? COL_DIM : COL_GRID, 1);
    }

    float fill = clampf(heave_norm, -1, 1);
    float max_b = h * 0.5f - 3;
    float bh = fabsf(fill) * max_b;
    ImU32 col = severity_color(fabsf(fill));
    ImU32 gc  = (col & 0x00FFFFFF) | 0x22000000;

    if (fill > 0.02f) {
        dl->AddRectFilled(ImVec2(x + 3, cy - bh), ImVec2(x + w - 3, cy),
                          col, 2);
        dl->AddRectFilled(ImVec2(x + 1, cy - bh - 2), ImVec2(x + w - 1, cy + 2),
                          gc, 3);
    } else if (fill < -0.02f) {
        dl->AddRectFilled(ImVec2(x + 3, cy), ImVec2(x + w - 3, cy + bh),
                          col, 2);
        dl->AddRectFilled(ImVec2(x + 1, cy - 2), ImVec2(x + w - 1, cy + bh + 2),
                          gc, 3);
    }

    float py = cy - fill * max_b;
    dl->AddTriangleFilled(ImVec2(x + w + 2, py),
                          ImVec2(x + w + 8, py - 4),
                          ImVec2(x + w + 8, py + 4), col);

    char vb[16];
    snprintf(vb, sizeof(vb), "%+d", heave_raw);
    ImVec2 vs = ImGui::CalcTextSize(vb);
    dl->AddText(ImVec2(x + (w - vs.x) * 0.5f, y + h + 5), COL_TEXT, vb);

    const char *lbl = "DEPTH";
    ImVec2 ls = ImGui::CalcTextSize(lbl);
    dl->AddText(ImVec2(x + (w - ls.x) * 0.5f, y - 16), COL_DIM, lbl);
}

/* ================================================================== */
/*  Camera viewport background (ocean placeholder)                    */
/* ================================================================== */

static void render_camera_background(ImDrawList *dl, ImVec2 pos, ImVec2 sz)
{
    float x = pos.x, y = pos.y, w = sz.x, h = sz.y;
    float t = (float)SDL_GetTicks() * 0.001f;

    /* 1. Tropical water gradient — light turquoise at top, deeper blue below */
    int bands = 16;
    for (int i = 0; i < bands; i++) {
        float t0 = (float)i / (float)bands;
        float t1 = (float)(i + 1) / (float)bands;
        /* Top: bright turquoise (#68D8D8), Bottom: ocean blue (#3498C8) */
        int r0 = (int)(104 - t0 * 52);
        int g0 = (int)(216 - t0 * 64);
        int b0 = (int)(216 - t0 * 16);
        int r1 = (int)(104 - t1 * 52);
        int g1 = (int)(216 - t1 * 64);
        int b1 = (int)(216 - t1 * 16);

        dl->AddRectFilledMultiColor(
            ImVec2(x, y + t0 * h), ImVec2(x + w, y + t1 * h),
            IM_COL32(r0, g0, b0, 255), IM_COL32(r0, g0, b0, 255),
            IM_COL32(r1, g1, b1, 255), IM_COL32(r1, g1, b1, 255));
    }

    /* 2. Animated sun-caustic light patches */
    for (int i = 0; i < 5; i++) {
        float rx = x + w * (0.12f + 0.20f * (float)i
                            + 0.04f * sinf(t * 0.25f + (float)i * 1.4f));
        float rw_top = 8.0f + 4.0f * sinf(t * 0.4f + (float)i);
        float rw_bot = 40.0f + 15.0f * sinf(t * 0.35f + (float)i * 0.7f);
        float rh = h * (0.35f + 0.12f * sinf(t * 0.18f + (float)i));
        int alpha = (int)(18 + 10 * sinf(t * 0.5f + (float)i));

        ImVec2 pts[4] = {
            ImVec2(rx - rw_top, y),
            ImVec2(rx + rw_top, y),
            ImVec2(rx + rw_bot, y + rh),
            ImVec2(rx - rw_bot, y + rh),
        };
        dl->AddConvexPolyFilled(pts, 4, IM_COL32(255, 255, 255, alpha));
    }

    /* 3. Floating bright particles (bubbles / sparkles) */
    for (int i = 0; i < 14; i++) {
        float px = x + w * fmodf(0.075f * (float)i
                   + sinf(t * 0.12f + (float)i * 0.6f) * 0.04f + 0.5f, 1.0f);
        float py = y + h * fmodf(0.085f * (float)i
                   + t * 0.018f * (1.0f + (float)i * 0.08f), 1.0f);
        float ps = 1.2f + 0.5f * sinf(t + (float)i);
        int   pa = 40 + (int)(20 * sinf(t * 1.8f + (float)i));
        dl->AddCircleFilled(ImVec2(px, py), ps,
                            IM_COL32(255, 255, 255, pa));
    }

    /* 4. Subtle scan lines (very faint) */
    for (float sy = y; sy < y + h; sy += 4.0f)
        dl->AddLine(ImVec2(x, sy), ImVec2(x + w, sy),
                    IM_COL32(255, 255, 255, 10), 1.0f);

    /* 5. Soft bright vignette — lighten edges gently */
    float v = 40.0f;
    dl->AddRectFilledMultiColor(
        ImVec2(x, y), ImVec2(x + w, y + v),
        IM_COL32(180,235,240,45), IM_COL32(180,235,240,45),
        IM_COL32(180,235,240,0),  IM_COL32(180,235,240,0));
    dl->AddRectFilledMultiColor(
        ImVec2(x, y + h - v), ImVec2(x + w, y + h),
        IM_COL32(40,100,150,0),  IM_COL32(40,100,150,0),
        IM_COL32(40,100,150,30), IM_COL32(40,100,150,30));

    /* 6. Viewfinder corner brackets */
    float cn = 28.0f, ct = 2.0f;
    ImU32 fc = COL_CAM_FRAME;
    dl->AddLine(ImVec2(x+5, y+5), ImVec2(x+cn, y+5), fc, ct);
    dl->AddLine(ImVec2(x+5, y+5), ImVec2(x+5, y+cn), fc, ct);

    dl->AddLine(ImVec2(x+w-cn, y+5), ImVec2(x+w-5, y+5), fc, ct);
    dl->AddLine(ImVec2(x+w-5, y+5),  ImVec2(x+w-5, y+cn), fc, ct);

    dl->AddLine(ImVec2(x+5, y+h-5),  ImVec2(x+cn, y+h-5), fc, ct);
    dl->AddLine(ImVec2(x+5, y+h-cn), ImVec2(x+5, y+h-5),  fc, ct);

    dl->AddLine(ImVec2(x+w-cn, y+h-5), ImVec2(x+w-5, y+h-5), fc, ct);
    dl->AddLine(ImVec2(x+w-5, y+h-cn), ImVec2(x+w-5, y+h-5), fc, ct);

    /* 7. Centre crosshair (faint) */
    float ccx = x + w * 0.5f, ccy = y + h * 0.5f;
    ImU32 chc = IM_COL32(255, 255, 255, 60);
    dl->AddLine(ImVec2(ccx - 22, ccy), ImVec2(ccx - 5, ccy), chc, 1);
    dl->AddLine(ImVec2(ccx + 5,  ccy), ImVec2(ccx + 22, ccy), chc, 1);
    dl->AddLine(ImVec2(ccx, ccy - 22), ImVec2(ccx, ccy - 5), chc, 1);
    dl->AddLine(ImVec2(ccx, ccy + 5),  ImVec2(ccx, ccy + 22), chc, 1);

    /* 8. "CAM 1" indicator (top-left) */
    float pulse = 0.4f + 0.6f * sinf(t * 2.2f);
    dl->AddCircleFilled(ImVec2(x + 18, y + 18), 4,
                        IM_COL32(255, 107, 107, (int)(pulse * 200)));
    dl->AddText(ImVec2(x + 28, y + 12), IM_COL32(255, 255, 255, 200),
                "CAM 1");

    /* 9. "AWAITING VIDEO FEED" placeholder text (subtle) */
    const char *ns = "AWAITING VIDEO FEED";
    ImVec2 nsz = ImGui::CalcTextSize(ns);
    dl->AddText(ImVec2(x + (w - nsz.x) * 0.5f, y + h * 0.82f),
                IM_COL32(255, 255, 255, 80), ns);
}

/* ================================================================== */
/*  Panel: Title bar                                                  */
/* ================================================================== */

static void render_title_bar(const gui_frame_t *d, float w)
{
    ImDrawList *dl = ImGui::GetWindowDrawList();
    ImVec2 wp = ImGui::GetCursorScreenPos();

    /* Light title bar with warm bottom gradient */
    dl->AddRectFilledMultiColor(ImVec2(0, 0), ImVec2(w, 40),
        IM_COL32(240, 249, 252, 255), IM_COL32(240, 249, 252, 255),
        IM_COL32(218, 238, 248, 255), IM_COL32(218, 238, 248, 255));
    /* Thin colourful accent line along bottom */
    for (float gx = 0; gx < w; gx += 1.0f) {
        float frac = gx / w;
        int alpha = (int)(100.0f * (1.0f - fabsf(frac - 0.5f) * 2.0f));
        /* Rainbow-ish: teal → cyan → blue */
        int r = (int)(0 + frac * 52);
        int g = (int)(190 + frac * 30);
        int b = (int)(200 + frac * 40);
        dl->AddLine(ImVec2(gx, 39), ImVec2(gx + 1, 39),
                    IM_COL32(r, g, b, alpha), 2.0f);
    }

    /* Logo dot — coral/orange for warmth */
    ImVec2 dot(wp.x + 10, wp.y + 8);
    dl->AddCircleFilled(dot, 9, IM_COL32(255, 140, 66, 30));
    dl->AddCircleFilled(dot, 5, COL_ORANGE);
    dl->AddCircle(dot, 5, IM_COL32(255, 255, 255, 80), 0, 1);

    ImGui::SetCursorPosX(28);
    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.10f, 0.23f, 0.36f, 1.0f));
    ImGui::SetWindowFontScale(1.15f);
    ImGui::Text("KAYRA ROV Surface Software");
    ImGui::SetWindowFontScale(1.0f);
    ImGui::PopStyleColor();

    ImGui::SameLine(w * 0.45f);
    ImGui::TextDisabled("%s", d->transport_str ? d->transport_str : "---");

    /* Status */
    ImGui::SameLine(w - 400);
    ImVec2 ip = ImGui::GetCursorScreenPos();
    if (d->failsafe) {
        float p = 0.5f + 0.5f * sinf((float)SDL_GetTicks() * 0.006f);
        dl->AddCircleFilled(ImVec2(ip.x - 14, ip.y + 8), 4,
                            IM_COL32(235, 77, 75, (int)(p * 255)));
        dl->AddCircleFilled(ImVec2(ip.x - 14, ip.y + 8), 8,
                            IM_COL32(235, 77, 75, (int)(p * 35)));
        ImGui::PushStyleColor(ImGuiCol_Text,
                              ImVec4(0.922f, 0.302f, 0.294f, 1.0f));
        ImGui::Text("FAILSAFE");
        ImGui::PopStyleColor();
    } else {
        dl->AddCircleFilled(ImVec2(ip.x - 14, ip.y + 8), 4, COL_GREEN);
        dl->AddCircleFilled(ImVec2(ip.x - 14, ip.y + 8), 7, COL_GREEN_DIM);
        ImGui::PushStyleColor(ImGuiCol_Text,
                              ImVec4(0.180f, 0.800f, 0.443f, 1.0f));
        ImGui::Text("NOMINAL");
        ImGui::PopStyleColor();
    }

    /* ARM/DISARM badge in title bar */
    ImGui::SameLine();
    ImGui::Text("  ");
    ImGui::SameLine();
    if (d->rov_connected) {
        if (d->rov_armed) {
            ImGui::PushStyleColor(ImGuiCol_Text,
                                  ImVec4(0.16f, 0.78f, 0.31f, 1.0f));
            ImGui::Text("ARMED");
            ImGui::PopStyleColor();
        } else {
            ImGui::PushStyleColor(ImGuiCol_Text,
                                  ImVec4(0.78f, 0.63f, 0.16f, 1.0f));
            ImGui::Text("DISARMED");
            ImGui::PopStyleColor();
        }
    } else {
        ImGui::TextDisabled("NO ROV");
    }

    ImGui::SameLine(w - 155);
    ImGui::TextDisabled("%.0f Hz | %lu pkts",
                        d->loop_hz, (unsigned long)d->packets_sent);

    ImGui::SetCursorPosY(44);
}

/* ================================================================== */
/*  Panel: Left — sticks, buttons, status                             */
/* ================================================================== */

static void render_left_panel(const gui_frame_t *d,
                               float panel_w, float panel_h)
{
    ImGui::BeginChild("##left", ImVec2(panel_w, panel_h), false);

    ImDrawList *dl = ImGui::GetWindowDrawList();
    ImVec2 wp    = ImGui::GetCursorScreenPos();
    ImVec2 avail = ImGui::GetContentRegionAvail();

    float sr = (avail.x - 36) * 0.5f;
    if (sr > 64) sr = 64;
    if (sr < 34) sr = 34;
    float cx = wp.x + avail.x * 0.5f;

    dl->AddText(ImVec2(wp.x + 6, wp.y + 3), COL_DIM, "JOYSTICK");

    /* Left stick — Sway (X) + Surge (Y) — movement */
    float lx = d->js_num_axes > 0 ? d->js_axes[0] : 0;
    float ly = d->js_num_axes > 1 ? d->js_axes[1] : 0;
    draw_circular_stick(dl, ImVec2(cx, wp.y + 28 + sr), sr, lx, ly,
                        "Sway / Surge");

    /* Right stick — Yaw (X) + Heave (Y) — rotation + vertical */
    float rx = d->js_num_axes > 2 ? d->js_axes[2] : 0;
    float ry = d->js_num_axes > 3 ? d->js_axes[3] : 0;
    float r2y = wp.y + 28 + sr + sr + 48 + sr;
    draw_circular_stick(dl, ImVec2(cx, r2y), sr, rx, ry, "Yaw / Heave");

    /* Buttons */
    float by = r2y + sr + 42;
    dl->AddText(ImVec2(wp.x + 6, by), COL_DIM, "BUTTONS");
    by += 16;

    int show = d->js_num_buttons;
    if (show <= 0) show = 16;
    if (show > 16) show = 16;
    float bs = 17, bg = 2;

    for (int i = 0; i < show; i++) {
        float bx = wp.x + 6 + (float)(i % 8) * (bs + bg);
        float bby = by + (float)(i / 8) * (bs + bg);
        bool on = (d->js_buttons >> i) & 1;
        dl->AddRectFilled(ImVec2(bx, bby), ImVec2(bx + bs, bby + bs),
                          on ? COL_CYAN : IM_COL32(220, 236, 246, 255), 3);
        dl->AddRect(ImVec2(bx, bby), ImVec2(bx + bs, bby + bs),
                    on ? COL_CYAN : COL_BORDER, 3);
        if (on)
            dl->AddRectFilled(ImVec2(bx - 1, bby - 1),
                              ImVec2(bx + bs + 1, bby + bs + 1),
                              IM_COL32(0, 206, 209, 30), 4);
        char nb[4];
        snprintf(nb, sizeof(nb), "%d", i);
        ImVec2 ns = ImGui::CalcTextSize(nb);
        dl->AddText(ImVec2(bx + (bs - ns.x) * 0.5f,
                           bby + (bs - ns.y) * 0.5f),
                    on ? IM_COL32(255, 255, 255, 255) : COL_DIM, nb);
    }

    /* Status */
    float sy = by + (float)((show + 7) / 8) * (bs + bg) + 14;
    dl->AddLine(ImVec2(wp.x + 6, sy), ImVec2(wp.x + avail.x - 6, sy),
                COL_BORDER, 1);
    sy += 6;
    dl->AddText(ImVec2(wp.x + 6, sy), COL_DIM, "STATUS");
    sy += 16;

    if (d->js_connected) {
        dl->AddCircleFilled(ImVec2(wp.x + 14, sy + 7), 4, COL_GREEN);
        dl->AddCircleFilled(ImVec2(wp.x + 14, sy + 7), 7, COL_GREEN_DIM);
        dl->AddText(ImVec2(wp.x + 24, sy), COL_TEXT, "Joystick OK");
    } else {
        float p = 0.5f + 0.5f * sinf((float)SDL_GetTicks() * 0.005f);
        dl->AddCircleFilled(ImVec2(wp.x + 14, sy + 7), 4,
                            IM_COL32(235, 77, 75, (int)(p * 255)));
        dl->AddText(ImVec2(wp.x + 24, sy), COL_RED, "No Joystick");
    }
    sy += 17;

    dl->AddCircleFilled(ImVec2(wp.x + 14, sy + 7), 4, COL_ACCENT);
    char tb[72];
    snprintf(tb, sizeof(tb), "TX: %s",
             d->transport_str ? d->transport_str : "N/A");
    dl->AddText(ImVec2(wp.x + 24, sy), COL_TEXT, tb);
    sy += 17;

    if (d->failsafe) {
        dl->AddCircleFilled(ImVec2(wp.x + 14, sy + 7), 4, COL_RED);
        dl->AddText(ImVec2(wp.x + 24, sy), COL_RED, "FAILSAFE");
    } else {
        dl->AddCircleFilled(ImVec2(wp.x + 14, sy + 7), 4, COL_GREEN);
        dl->AddText(ImVec2(wp.x + 24, sy), COL_TEXT, "Normal");
    }
    sy += 19;

    /* ROV connection + armed state */
    {
        ImU32 conn_col = d->rov_connected ? COL_GREEN : COL_RED;
        const char *conn_txt = d->rov_connected ? "ROV ONLINE" : "ROV OFFLINE";
        dl->AddCircleFilled(ImVec2(wp.x + 14, sy + 7), 4, conn_col);
        dl->AddText(ImVec2(wp.x + 24, sy), conn_col, conn_txt);
        sy += 17;

        if (d->rov_connected) {
            if (d->rov_armed) {
                float p = 0.5f + 0.5f * sinf((float)SDL_GetTicks() * 0.004f);
                ImU32 arm_col = IM_COL32(40, 200, 80, (int)(180 + p * 75));
                dl->AddCircleFilled(ImVec2(wp.x + 14, sy + 7), 5, arm_col);
                dl->AddText(ImVec2(wp.x + 24, sy), IM_COL32(40, 200, 80, 255),
                            "ARMED");
            } else {
                dl->AddCircleFilled(ImVec2(wp.x + 14, sy + 7), 4,
                                    IM_COL32(200, 160, 40, 200));
                dl->AddText(ImVec2(wp.x + 24, sy), IM_COL32(200, 160, 40, 255),
                            "DISARMED");
            }
        }
        sy += 19;
    }

    char sb[64];
    snprintf(sb, sizeof(sb), "Pkts:%lu  %.1fHz",
             (unsigned long)d->packets_sent, d->loop_hz);
    dl->AddText(ImVec2(wp.x + 10, sy), COL_DIM, sb);
    sy += 20;

    /* ── Battery ── */
    dl->AddLine(ImVec2(wp.x + 6, sy), ImVec2(wp.x + avail.x - 6, sy),
                COL_BORDER, 1);
    sy += 6;
    dl->AddText(ImVec2(wp.x + 6, sy), COL_DIM, "BATTERY");
    sy += 16;

    float bat_w = avail.x - 28;
    if (bat_w < 60) bat_w = 60;
    draw_battery_bar(dl, ImVec2(wp.x + 8, sy), bat_w, 24,
                     d->battery_voltage, d->battery_current,
                     d->battery_percent);
    sy += 42;

    /* ── Telemetry ── */
    dl->AddLine(ImVec2(wp.x + 6, sy), ImVec2(wp.x + avail.x - 6, sy),
                COL_BORDER, 1);
    sy += 6;
    dl->AddText(ImVec2(wp.x + 6, sy), COL_DIM, "TELEMETRY");
    sy += 16;

    {
        char t1[48], t2[48], t3[48];
        snprintf(t1, sizeof(t1), "Depth   %.1f m", d->depth_m);
        snprintf(t2, sizeof(t2), "Water   %.1f \xc2\xb0""C", d->water_temp_c);
        snprintf(t3, sizeof(t3), "Int.T   %.1f \xc2\xb0""C", d->internal_temp_c);

        ImU32 dtc = (d->depth_m > 80) ? COL_RED :
                    (d->depth_m > 50) ? COL_YELLOW : COL_TEXT;
        dl->AddText(ImVec2(wp.x + 10, sy), dtc, t1); sy += 15;
        dl->AddText(ImVec2(wp.x + 10, sy), COL_TEXT, t2); sy += 15;
        ImU32 itc = (d->internal_temp_c > 50) ? COL_RED :
                    (d->internal_temp_c > 40) ? COL_YELLOW : COL_TEXT;
        dl->AddText(ImVec2(wp.x + 10, sy), itc, t3); sy += 15;
    }

    ImGui::Dummy(ImVec2(avail.x, panel_h));
    ImGui::EndChild();
}

/* ================================================================== */
/*  Panel: Centre — camera viewport with HUD overlays                 */
/* ================================================================== */

static void render_center_panel(const gui_frame_t *d,
                                 float panel_w, float panel_h)
{
    ImGui::BeginChild("##center", ImVec2(panel_w, panel_h), false);

    ImDrawList *dl = ImGui::GetWindowDrawList();
    ImVec2 wp    = ImGui::GetCursorScreenPos();
    ImVec2 avail = ImGui::GetContentRegionAvail();
    float cam_x = wp.x, cam_y = wp.y;
    float cam_w = avail.x, cam_h = avail.y;

    /* ---- Camera feed or ocean placeholder ---- */
    if (d->camera_rgb && d->camera_w > 0 && d->camera_h > 0) {
        update_camera_texture(d->camera_rgb, d->camera_w, d->camera_h);
        if (s_cam_tex) {
            dl->AddImage((ImTextureID)(uintptr_t)s_cam_tex,
                         ImVec2(cam_x, cam_y),
                         ImVec2(cam_x + cam_w, cam_y + cam_h));
        }
    } else {
        render_camera_background(dl, ImVec2(cam_x, cam_y),
                                 ImVec2(cam_w, cam_h));
    }

    /* ---- HUD Overlay: Heading Tape (top) ---- */
    float tape_w = cam_w * 0.72f;
    float tape_h = 30.0f;
    float tape_x = cam_x + (cam_w - tape_w) * 0.5f;
    float tape_y = cam_y + 10;
    float heading_deg = (float)d->ctrl_r / 1000.0f * 180.0f;
    draw_heading_tape(dl, ImVec2(tape_x, tape_y), tape_w, tape_h,
                      heading_deg);

    float surge_n = (float)d->ctrl_x / 1000.0f;
    float sway_n  = (float)d->ctrl_y / 1000.0f;
    float heave_n = (float)d->ctrl_z / 1000.0f;

    /* ---- HUD Overlay: Attitude Indicator (bottom-left) ---- */
    float ai_r = cam_h * 0.22f;
    if (ai_r > 105) ai_r = 105;
    if (ai_r <  45) ai_r =  45;
    ImVec2 ai_ctr(cam_x + ai_r + 20, cam_y + cam_h - ai_r - 30);
    draw_attitude_indicator(dl, ai_ctr, ai_r, surge_n, sway_n);

    /* ---- HUD Overlay: Depth gauge (right edge) ---- */
    float dg_x = cam_x + cam_w - 34;
    float dg_y = cam_y + tape_h + 55;
    float dg_h = cam_h - tape_h - 110;
    if (dg_h < 80) dg_h = 80;
    draw_depth_gauge(dl, ImVec2(dg_x, dg_y), 18, dg_h, heave_n, d->ctrl_z);

    /* ---- HUD Overlay: 3-D ROV model (bottom-right) ---- */
    float rov_r = ai_r * 0.52f;
    if (rov_r < 38) rov_r = 38;
    if (rov_r > 80) rov_r = 80;
    ImVec2 rov_ctr(cam_x + cam_w - rov_r - 42, cam_y + cam_h - rov_r - 32);
    draw_rov_3d(dl, rov_ctr, rov_r,
                d->imu_roll, d->imu_pitch, d->imu_yaw);

    /* ---- HUD Overlay: Bottom info strip ---- */
    {
        float strip_h = 22;
        float strip_w = cam_w * 0.55f;
        float sx = cam_x + (cam_w - strip_w) * 0.5f;
        float sy = cam_y + cam_h - strip_h - 6;

        dl->AddRectFilled(ImVec2(sx, sy),
                          ImVec2(sx + strip_w, sy + strip_h),
                          IM_COL32(255, 255, 255, 180), 3);
        dl->AddRect(ImVec2(sx, sy),
                    ImVec2(sx + strip_w, sy + strip_h),
                    COL_BORDER, 3);

        char info[128];
        snprintf(info, sizeof(info),
                 "SRG:%+5d   SWY:%+5d   HVE:%+5d   YAW:%+5d",
                 d->ctrl_x, d->ctrl_y, d->ctrl_z, d->ctrl_r);
        ImVec2 isz = ImGui::CalcTextSize(info);
        dl->AddText(ImVec2(sx + (strip_w - isz.x) * 0.5f, sy + 4),
                    COL_TEXT, info);
    }

    /* Camera frame border — teal outline */
    dl->AddRect(ImVec2(cam_x, cam_y), ImVec2(cam_x + cam_w, cam_y + cam_h),
                IM_COL32(0, 180, 200, 120), 0, 0, 2.0f);
    /* Inner subtle white glow */
    dl->AddRect(ImVec2(cam_x + 1, cam_y + 1),
                ImVec2(cam_x + cam_w - 1, cam_y + cam_h - 1),
                IM_COL32(255, 255, 255, 40), 0, 0, 3.0f);

    ImGui::Dummy(avail);
    ImGui::EndChild();
}

/* ================================================================== */
/*  Panel: Right — arc gauges                                         */
/* ================================================================== */

static void render_right_panel(const gui_frame_t *d,
                                float panel_w, float panel_h)
{
    ImGui::BeginChild("##right", ImVec2(panel_w, panel_h), false);

    ImDrawList *dl = ImGui::GetWindowDrawList();
    ImVec2 wp    = ImGui::GetCursorScreenPos();
    ImVec2 avail = ImGui::GetContentRegionAvail();

    dl->AddText(ImVec2(wp.x + 6, wp.y + 3), COL_DIM, "CONTROL CHANNELS");

    /* 2×2 gauge grid */
    float gr = (avail.x - 44) * 0.25f;
    if (gr > (avail.y - 60) * 0.22f) gr = (avail.y - 60) * 0.22f;
    if (gr > 68) gr = 68;
    if (gr < 34) gr = 34;

    float c1 = wp.x + avail.x * 0.30f;
    float c2 = wp.x + avail.x * 0.70f;
    float r1 = wp.y + 26 + gr + 6;
    float r2 = r1 + gr * 2.5f;

    draw_arc_gauge(dl, ImVec2(c1, r1), gr,
                   (float)d->ctrl_x, -1000, 1000, "SURGE", COL_CYAN);
    draw_arc_gauge(dl, ImVec2(c2, r1), gr,
                   (float)d->ctrl_y, -1000, 1000, "SWAY", COL_ORANGE);
    draw_arc_gauge(dl, ImVec2(c1, r2), gr,
                   (float)d->ctrl_z, -1000, 1000, "HEAVE", COL_GREEN);
    draw_arc_gauge(dl, ImVec2(c2, r2), gr,
                   (float)d->ctrl_r, -1000, 1000, "YAW", COL_PURPLE);

    ImGui::Dummy(ImVec2(avail.x, panel_h));
    ImGui::EndChild();
}

/* ================================================================== */
/*  Public API                                                        */
/* ================================================================== */

extern "C" int gui_init(int width, int height)
{
    if (SDL_InitSubSystem(SDL_INIT_VIDEO | SDL_INIT_TIMER) < 0) {
        fprintf(stderr, "[gui] SDL video init failed: %s\n", SDL_GetError());
        return -1;
    }

    SDL_GL_SetAttribute(SDL_GL_CONTEXT_FLAGS, 0);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK,
                        SDL_GL_CONTEXT_PROFILE_CORE);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 0);
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);
    SDL_GL_SetAttribute(SDL_GL_STENCIL_SIZE, 8);

    s_win = SDL_CreateWindow(
        "KAYRA ROV \xe2\x80\x94 Surface Software",
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        width, height,
        SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI);
    if (!s_win) {
        fprintf(stderr, "[gui] Window creation failed: %s\n", SDL_GetError());
        return -1;
    }

    s_gl = SDL_GL_CreateContext(s_win);
    if (!s_gl) {
        fprintf(stderr, "[gui] GL context failed: %s\n", SDL_GetError());
        return -1;
    }
    SDL_GL_MakeCurrent(s_win, s_gl);
    SDL_GL_SetSwapInterval(0);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO &io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    io.IniFilename = nullptr;

    ImFontConfig fc;
    fc.SizePixels = 16.0f;
    io.Fonts->AddFontDefault(&fc);

    apply_theme();

    ImGui_ImplSDL2_InitForOpenGL(s_win, s_gl);
    ImGui_ImplOpenGL3_Init(GLSL_VER);

    printf("[gui] KAYRA ROV UI %dx%d  GL %s\n", width, height,
           (const char *)glGetString(GL_VERSION));

    /* Load ROV 3-D model from STL */
    const char *stl_paths[] = {
        "assets/Kayra_ROV.stl",
        "./assets/Kayra_ROV.stl",
        nullptr,
    };
    for (int i = 0; stl_paths[i]; i++) {
        FILE *probe = fopen(stl_paths[i], "rb");
        if (probe) { fclose(probe); load_rov_stl(stl_paths[i]); break; }
    }

    return 0;
}

/* ------------------------------------------------------------------ */

extern "C" bool gui_render(const gui_frame_t *d)
{
    SDL_Event ev;
    while (SDL_PollEvent(&ev)) {
        ImGui_ImplSDL2_ProcessEvent(&ev);
        if (ev.type == SDL_QUIT) return false;
        if (ev.type == SDL_WINDOWEVENT &&
            ev.window.event == SDL_WINDOWEVENT_CLOSE &&
            ev.window.windowID == SDL_GetWindowID(s_win))
            return false;
    }

    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplSDL2_NewFrame();
    ImGui::NewFrame();

    int ww, wh;
    SDL_GetWindowSize(s_win, &ww, &wh);
    float w = (float)ww, h = (float)wh;

    ImGui::SetNextWindowPos(ImVec2(0, 0));
    ImGui::SetNextWindowSize(ImVec2(w, h));
    ImGui::Begin("##host", nullptr,
        ImGuiWindowFlags_NoTitleBar  | ImGuiWindowFlags_NoResize   |
        ImGuiWindowFlags_NoMove      | ImGuiWindowFlags_NoScrollbar |
        ImGuiWindowFlags_NoBringToFrontOnFocus);

    render_title_bar(d, w);

    float body_h   = h - 48;
    float left_w   = w * 0.18f;
    float right_w  = w * 0.30f;
    float center_w = w - left_w - right_w - 24;

    render_left_panel(d, left_w, body_h);
    ImGui::SameLine();
    render_center_panel(d, center_w, body_h);
    ImGui::SameLine();
    render_right_panel(d, right_w, body_h);

    ImGui::End();

    ImGui::Render();
    int dw, dh;
    SDL_GL_GetDrawableSize(s_win, &dw, &dh);
    glViewport(0, 0, dw, dh);
    glClearColor(0.878f, 0.949f, 0.973f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    SDL_GL_SwapWindow(s_win);

    return true;
}

/* ------------------------------------------------------------------ */

extern "C" void gui_shutdown(void)
{
    if (s_cam_tex) { glDeleteTextures(1, &s_cam_tex); s_cam_tex = 0; }
    if (s_rov_mesh) { delete[] s_rov_mesh; s_rov_mesh = nullptr; }

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplSDL2_Shutdown();
    ImGui::DestroyContext();

    if (s_gl)  { SDL_GL_DeleteContext(s_gl); s_gl  = nullptr; }
    if (s_win) { SDL_DestroyWindow(s_win);   s_win = nullptr; }

    printf("[gui] Shutdown complete.\n");
}
