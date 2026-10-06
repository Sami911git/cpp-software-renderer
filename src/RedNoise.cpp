#include <CanvasTriangle.h>
#include <DrawingWindow.h>
#include <Utils.h>
#include <CanvasPoint.h>
#include <Colour.h>
#include <TextureMap.h>
#include <TexturePoint.h>
#include <ModelTriangle.h>
#include <glm/glm.hpp>
#include <SDL.h>
#include <vector>
#include <unordered_map>
#include <string>
#include <sstream>
#include <fstream>
#include <iostream>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <ctime>
#include <RayTriangleIntersection.h>
#include <thread>

static int g_numThreads = 30;
#define WIDTH  320
#define HEIGHT 240



// for frame recording if true
static bool g_recordFrames = false; 
//  
// if u want to play the animation put true
static bool g_playIdent   = true;





static int g_frameIndex    = 0;

// globals,too many but it works
static std::vector<float> g_depth;

static const glm::vec3 GOLD_TINT(1.0f, 0.84f, 0.4f);

static float g_focusDepthZ    = 3.0f;

static const float CORNELL_RT_BRIGHTNESS = 1.7f;
const float PI = 3.14159265f;
static glm::vec3 g_sceneCentre  = glm::vec3(0.0f, 0.0f, 0.0f);
static glm::vec3 g_sphereCentre = glm::vec3(0.0f, 0.0f, 0.0f);

static const float LIGHT_RADIUS = 0.70f;
static const int   SHADOW_GRID  = 5;
static std::vector<uint32_t> g_colourBuffer;
static std::string g_ironMatName = "Blue";

enum IdentMatMode {
    IDENT_MAT_OFF,
    IDENT_MAT_NORMAL,
    IDENT_MAT_MIRROR_GLASS,
    IDENT_MAT_IRON_GOLD
};

static IdentMatMode g_identMatMode = IDENT_MAT_OFF;

// basic camera stuff
static glm::vec3 g_camera(0.0f, 0.0f, 0.0f);
static glm::mat3 g_camOrientation(1.0f);
static float     g_focal    = 2.0f;
static float     g_imgScale = 320.0f;

static const glm::vec3 g_sphereCamPos(0.0f, 0.0f, -4.0f);
static const float g_sphereBaseTiltRad = 0.0f * 0.05f;

static const float LIGHT_MOVE_STEP = 0.2f;

static glm::vec3 g_lightPos(5.0f, 9.0f, -9.0f);



enum SceneType {
    SCENE_SPHERE,
    SCENE_CORNELL,
    SCENE_ENV_SPHERE
};


// here which object to choose
static SceneType g_sceneType = SCENE_SPHERE;

static const float CAM_MOVE_STEP = 0.1f;
static const float CAM_ROT_STEP  = 0.05f;

enum DOFMode {
    DOF_OFF,
    DOF_PHYSICAL,
    DOF_OBJECT_LOCK
};

// here if i want the dof manual or automated
static DOFMode     g_dofMode         = DOF_OFF;
static std::string g_dofFocusMatName = "Blue";

static bool  g_enableDOF      = false;
static float g_focusDistance  = 3.0f;
static float g_apertureRadius = 0.0f;
static int   g_dofSamples     = 32;

static std::vector<ModelTriangle> g_model;
static std::unordered_map<std::string, Colour> g_palette;

static bool g_orbit         = false;
static bool g_isSphereScene = false;

enum RenderMode { MODE_NONE, MODE_WIREFRAME, MODE_RASTERISED, MODE_RAYTRACED };
static RenderMode g_mode = MODE_NONE;

// textures and env maps
static TextureMap g_floorTex("texture.ppm");

static TextureMap g_envMap("env.ppm");

static bool  g_floorBoundsInit = false;
static float g_floorMinX = 0.0f;
static float g_floorMaxX = 0.0f;
static float g_floorMinZ = 0.0f;
static float g_floorMaxZ = 0.0f;



// simple colour helpers and DOF presets etc

static bool isMetalTriangle(const ModelTriangle& mt) {
    switch (g_identMatMode) {
        case IDENT_MAT_NORMAL:
            return false;
        case IDENT_MAT_MIRROR_GLASS:
            return false;
        case IDENT_MAT_IRON_GOLD:
            return (mt.colour.name == "Red");
        case IDENT_MAT_OFF:
        default:
            return false;
    }
}

static bool isIronTriangle(const ModelTriangle& mt) {
    if (g_identMatMode == IDENT_MAT_IRON_GOLD) {
        return (mt.colour.name == g_ironMatName);
    }
    return false;
}



static bool isMirrorTriangle(const ModelTriangle& mt) {
    if (g_identMatMode == IDENT_MAT_MIRROR_GLASS) {
        return (mt.colour.name == "Red");
    }
    return false;
}

// here for the animation, 

static bool isRefractiveTriangle(const ModelTriangle& mt) {
    if (g_identMatMode == IDENT_MAT_NORMAL) {
        return false;
    }
    if (g_identMatMode == IDENT_MAT_MIRROR_GLASS) {
        return (mt.colour.name == "Blue");
    }
    if (g_identMatMode == IDENT_MAT_IRON_GOLD) {
        return false;
    }
    return false;
}

static bool isFloorTriangleWorld(const ModelTriangle& mt) {
    return (mt.colour.name == "Green");
}

// compute barycentric for point in world, used for normals
static void barycentricForPoint(const glm::vec3& A,const glm::vec3& B,const glm::vec3& C,const glm::vec3& P,float& b0, float& b1, float& b2){
    glm::vec3 v0 = B - A;
    glm::vec3 v1 = C - A;
    glm::vec3 v2 = P - A;

    float d00 = glm::dot(v0, v0);
    float d01 = glm::dot(v0, v1);
    float d11 = glm::dot(v1, v1);
    float d20 = glm::dot(v2, v0);
    float d21 = glm::dot(v2, v1);

    float denom = d00 * d11 - d01 * d01;
    if (std::abs(denom) < 1e-8f) {
        b0 = 1.0f; b1 = 0.0f; b2 = 0.0f;
        return;
    }

    float invDenom = 1.0f / denom;
    b1 = (d11 * d20 - d01 * d21) * invDenom;
    b2 = (d00 * d21 - d01 * d20) * invDenom;
    b0 = 1.0f - b1 - b2;
}

// ray triangle basic intersect, notoptimized
static RayTriangleIntersection getClosestIntersection(const glm::vec3& rayOrigin,const glm::vec3& rayDirection,int  triangleIndexToIgnore = -1,float maxDistance= std::numeric_limits<float>::infinity()) {
    RayTriangleIntersection closest;
    closest.distanceFromCamera = std::numeric_limits<float>::infinity();
    closest.triangleIndex      = -1;
    for (size_t i = 0; i < g_model.size(); ++i) {
        const ModelTriangle& tri = g_model[i];

        if ((int)i == triangleIndexToIgnore) continue;

        glm::vec3 e0 = tri.vertices[1] - tri.vertices[0];
        glm::vec3 e1 = tri.vertices[2] - tri.vertices[0];
        glm::vec3 SPVector = rayOrigin - tri.vertices[0];

        glm::mat3 DEMatrix(-rayDirection, e0, e1);

        float det = glm::determinant(DEMatrix);
        if (std::abs(det) < 1e-6f) continue;

        glm::vec3 solution = glm::inverse(DEMatrix) * SPVector;
        float t = solution.x;
        float u = solution.y;
        float v = solution.z;
        if (t <= 0.0f || t > maxDistance) continue;
        if (u < 0.0f || u > 1.0f)         continue;
        if (v < 0.0f || v > 1.0f)         continue;
        if (u + v > 1.0f)                 continue;

        if (t < closest.distanceFromCamera) {
            closest.distanceFromCamera  = t;
            closest.intersectionPoint   = rayOrigin + t * rayDirection;
            closest.intersectedTriangle = tri;
            closest.triangleIndex       = (int)i;
        }
    }

    return closest;
}

static void setSoftBlur() {
    g_enableDOF      = true;
    g_apertureRadius = 0.20f;
    g_dofSamples     = 32;
}

static void setStrongBlur() {
    g_enableDOF      = true;
    g_apertureRadius = 0.45f;
    g_dofSamples     = 64;
}

static inline uint32_t packARGB(uint8_t a, uint8_t r, uint8_t g, uint8_t b) {
    return (uint32_t(a) << 24) | (uint32_t(r) << 16) | (uint32_t(g) << 8) | uint32_t(b);
}

static inline uint32_t colourToARGB(const Colour& c) {
    return packARGB(255, (uint8_t)c.red, (uint8_t)c.green, (uint8_t)c.blue);
}

static inline Colour argbToColour(uint32_t argb) {
    uint8_t r = (argb >> 16) & 0xFF;
    uint8_t g = (argb >> 8)  & 0xFF;
    uint8_t b =  argb        & 0xFF;
    return Colour((int)r, (int)g, (int)b);
}

static inline void putColour(DrawingWindow& w, int x, int y, uint32_t argb) {
    const int W = (int)w.width;
    const int H = (int)w.height;

    if ((unsigned)x >= (unsigned)W || (unsigned)y >= (unsigned)H) return;

    w.setPixelColour(x, y, argb);

    if ((int)g_colourBuffer.size() == W * H) {
        g_colourBuffer[(size_t)y * W + x] = argb;
    }
}

static inline uint32_t sampleTextureARGB(const TextureMap& tex, int u, int v) {
    if (u < 0) u = 0;
    if (v < 0) v = 0;
    if (u >= tex.width)  u = tex.width  - 1;
    if (v >= tex.height) v = tex.height - 1;

    uint32_t rgb = tex.pixels[(size_t)v * tex.width + u];
    return 0xFF000000u | (rgb & 0x00FFFFFFu);
}

static inline void putIfNearer(DrawingWindow& w, int x, int y, float invZ, uint32_t argb) {
    const int W = (int)w.width;
    const int H = (int)w.height;
    if ((unsigned)x >= (unsigned)W || (unsigned)y >= (unsigned)H) return;

    float &slot = g_depth[(size_t)y * W + x];
    if (invZ > slot) {
        slot = invZ;
        putColour(w, x, y, argb);
    }
}

// small lighting helper, a bit hacked
static float computeVertexBrightness(const glm::vec3& P,const glm::vec3& normal){
    glm::vec3 N = glm::normalize(normal);
    glm::vec3 L = glm::normalize(g_lightPos - P);
    glm::vec3 V = glm::normalize(g_camera - P);

    const float ambient      = 0.05f;
    const float diffStrength = 1.0f;
    const float specStrength = 0.6f;
    const float shininess    = 64.0f;

    float nDotL = glm::dot(N, L);
    if (nDotL < 0.0f) nDotL = 0.0f;
    float diffuse = diffStrength * nDotL;

    float specular = 0.0f;
    if (nDotL > 0.0f) {
        glm::vec3 R = glm::reflect(-L, N);
        float rDotV = glm::dot(R, V);
        if (rDotV > 0.0f) {
            specular = specStrength * std::pow(rDotV, shininess);
        }
    }

    float brightness = ambient + diffuse + specular;
    return glm::clamp(brightness, 0.0f, 1.0f);
}
static inline float randFloat(float minVal, float maxVal) {
    float t = (float)std::rand() / (float)RAND_MAX;
    return minVal + t * (maxVal - minVal);
}

static glm::vec3 perturbDirection(const glm::vec3& R, float roughness) {
    glm::vec3 dir = glm::normalize(R);

    glm::vec3 up = (std::fabs(dir.y) < 0.999f) ? glm::vec3(0.0f, 1.0f, 0.0f)
                                               : glm::vec3(1.0f, 0.0f, 0.0f);
    glm::vec3 tangent   = glm::normalize(glm::cross(up, dir));
    glm::vec3 bitangent = glm::normalize(glm::cross(dir, tangent));

    float theta = randFloat(0.0f, 2.0f * PI);
    float r= randFloat(0.0f, roughness);
    float x = r * std::cos(theta);
    float y = r * std::sin(theta);
    glm::vec3 offset = x * tangent + y * bitangent;
    return glm::normalize(dir + offset);
}

static glm::vec3 sampleApertureOffsetCam(float radius) {
    float u     = randFloat(0.0f, 1.0f);
    float v     = randFloat(0.0f, 1.0f);
    float r     = radius * std::sqrt(u);
    float theta = 2.0f * PI * v;

    float dx = r * std::cos(theta);
    float dy= r * std::sin(theta);

    return glm::vec3(dx, dy, 0.0f);
}

// this picks smoother sphere normals for sphere mesh
static glm::vec3 shadingNormal(const ModelTriangle& tri,const glm::vec3& P){
    if (tri.colour.name == "RedSphere") {
        glm::vec3 A= tri.vertices[0];
        glm::vec3 B=tri.vertices[1];
        glm::vec3 C= tri.vertices[2];

        float b0, b1, b2;
        barycentricForPoint(A, B, C, P, b0, b1, b2);

        glm::vec3 N0 = glm::normalize(A - g_sphereCentre);
        glm::vec3 N1 = glm::normalize(B - g_sphereCentre);
        glm::vec3 N2 = glm::normalize(C - g_sphereCentre);

        glm::vec3 N = b0 * N0 + b1 * N1 + b2 * N2;
        return glm::normalize(N);
    }
    return glm::normalize(tri.normal);
}

// simple gouraud triangle (z-buffer)
static void drawGouraudTriangleZ(DrawingWindow& w,const CanvasTriangle& tri,const Colour& baseCol){
    const CanvasPoint& a = tri.vertices[0];
    const CanvasPoint& b = tri.vertices[1];
    const CanvasPoint& c = tri.vertices[2];

    float minX = std::min({ a.x, b.x, c.x });
    float maxX = std::max({ a.x, b.x, c.x });
    float minY = std::min({ a.y, b.y, c.y });
    float maxY = std::max({ a.y, b.y, c.y });

    int x0 = std::max(0, (int)std::floor(minX));
    int x1 = std::min((int)w.width  - 1, (int)std::ceil(maxX));
    int y0 = std::max(0, (int)std::floor(minY));
    int y1 = std::min((int)w.height - 1, (int)std::ceil(maxY));

    auto edge = [](const CanvasPoint& p0, const CanvasPoint& p1, float x, float y) {
        return (x - p0.x) * (p1.y - p0.y) - (y - p0.y) * (p1.x - p0.x);
    };

    float area = edge(a, b, c.x, c.y);
    if (area == 0.0f) return;
    float invArea = 1.0f / area;

    const float CORNELL_BRIGHTNESS_BOOST = 1.3f;

    for (int y = y0; y <= y1; ++y) {
        for (int x = x0; x <= x1; ++x) {
            float px = x + 0.5f;
            float py = y + 0.5f;

            float w0 = edge(b, c, px, py);
            float w1 = edge(c, a, px, py);
            float w2 = edge(a, b, px, py);

            bool inside =
                (area > 0 && w0 >= 0 && w1 >= 0 && w2 >= 0) ||
                (area < 0 && w0 <= 0 && w1 <= 0 && w2 <= 0);
            if (!inside) continue;

            float l0 = w0 * invArea;
            float l1 = w1 * invArea;
            float l2 = w2 * invArea;

            float invZ = l0 * a.depth + l1 * b.depth + l2 * c.depth;
            float B    = l0 * a.brightness + l1 * b.brightness + l2 * c.brightness;

            if (!g_isSphereScene) {
                B *= CORNELL_BRIGHTNESS_BOOST;
            }

            B = glm::clamp(B, 0.0f, 1.0f);

            Colour shaded;
            shaded.red   = (int)(baseCol.red   * B);
            shaded.green = (int)(baseCol.green * B);
            shaded.blue  = (int)(baseCol.blue  * B);

            uint32_t argb = colourToARGB(shaded);
            putIfNearer(w, x, y, invZ, argb);
        }
    }
}

// phong per-pixel on triangle
static Colour phongShade(const glm::vec3& P, const glm::vec3& normal,const Colour& baseCol){
    glm::vec3 N = glm::normalize(normal);
    glm::vec3 L = glm::normalize(g_lightPos - P);
    glm::vec3 V = glm::normalize(g_camera  - P);

    const float ambient      = 0.05f;
    const float diffStrength = 1.0f;
    const float specStrength = 1.5f;
    const float shininess    = 64.0f;

    float nDotL = glm::dot(N, L);
    if (nDotL < 0.0f) nDotL = 0.0f;
    float diffuse = diffStrength * nDotL;

    float specular = 0.0f;
    if (nDotL > 0.0f) {
        glm::vec3 H = glm::normalize(L + V);
        float nDotH = glm::dot(N, H);
        if (nDotH > 0.0f) {
            specular = specStrength * std::pow(nDotH, shininess);
        }
    }

    glm::vec3 base(
        baseCol.red   / 255.0f,
        baseCol.green / 255.0f,
        baseCol.blue  / 255.0f
    );

    glm::vec3 col = base * (ambient + diffuse) + glm::vec3(1.0f) * specular;
    col = glm::clamp(col, glm::vec3(0.0f), glm::vec3(1.0f));

    Colour out;
    out.red   = (int)std::round(col.r * 255.0f);
    out.green = (int)std::round(col.g * 255.0f);
    out.blue  = (int)std::round(col.b * 255.0f);
    return out;
}

static void drawPhongTriangleZ(DrawingWindow& w,const CanvasTriangle& tri,const glm::vec3& P0,
                               const glm::vec3& P1,const glm::vec3& P2,
                               const glm::vec3& N0,const glm::vec3& N1,
                               const glm::vec3& N2,const Colour& baseCol){
    const CanvasPoint& a = tri.vertices[0];
    const CanvasPoint& b = tri.vertices[1];
    const CanvasPoint& c = tri.vertices[2];

    float minX = std::min({ a.x, b.x, c.x });
    float maxX = std::max({ a.x, b.x, c.x });
    float minY = std::min({ a.y, b.y, c.y });
    float maxY = std::max({ a.y, b.y, c.y});

    int x0 = std::max(0, (int)std::floor(minX));
    int x1 = std::min((int)w.width  - 1, (int)std::ceil(maxX));
    int y0 = std::max(0, (int)std::floor(minY));
    int y1 = std::min((int)w.height - 1, (int)std::ceil(maxY));

    auto edge = [](const CanvasPoint& p0, const CanvasPoint& p1, float x, float y) {
        return (x - p0.x) * (p1.y - p0.y) - (y - p0.y) * (p1.x - p0.x);
    };

    float area = edge(a, b, c.x, c.y);
    if (area == 0.0f) return;
    float invArea = 1.0f / area;

    for (int y = y0; y <= y1; ++y) {
        for (int x = x0; x <= x1; ++x) {
            float px = x + 0.5f;
            float py = y + 0.5f;

            float w0 = edge(b, c, px, py);
            float w1 = edge(c, a, px, py);
            float w2 = edge(a, b, px, py);

            bool inside =
                (area > 0 && w0 >= 0 && w1 >= 0 && w2 >= 0) ||
                (area < 0 && w0 <= 0 && w1 <= 0 && w2 <= 0);

            if (!inside) continue;

            float l0 = w0 * invArea;
            float l1 = w1 * invArea;
            float l2 = w2 * invArea;

            float invZ = l0 * a.depth + l1 * b.depth + l2 * c.depth;

            glm::vec3 P = l0 * P0 + l1 * P1 + l2 * P2;
            glm::vec3 N = glm::normalize(l0 * N0 + l1 * N1 + l2 * N2);

            Colour shaded = phongShade(P, N, baseCol);

            uint32_t argb = colourToARGB(shaded);
            putIfNearer(w, x, y, invZ, argb);
        }
    }
}

// wireframe for debugging / marking
static void drawLine(DrawingWindow& w, const CanvasPoint& a, const CanvasPoint& b, const Colour& col) {
    float x0 = a.x, y0 = a.y;
    float x1 = b.x, y1 = b.y;
    float dx = x1 - x0;
    float dy = y1 - y0;
    int steps = (int)std::max(std::fabs(dx), std::fabs(dy));
    uint32_t argb = colourToARGB(col);

    if (steps <= 0) {
        int xi = (int)std::lround(x0);
        int yi = (int)std::lround(y0);
        if (xi >= 0 && xi < w.width && yi >= 0 && yi < w.height)
            w.setPixelColour(xi, yi, argb);
        return;
    }

    float sx = dx / steps;
    float sy = dy / steps;
    float x = x0, y = y0;
    for (int i = 0; i <= steps; ++i) {
        int xi = (int)std::lround(x);
        int yi = (int)std::lround(y);
        if (xi >= 0 && xi < w.width && yi >= 0 && yi < w.height)
            w.setPixelColour(xi, yi, argb);
        x += sx;
        y += sy;
    }
}

static void drawStrokedTriangle(DrawingWindow& w, const CanvasTriangle& t, const Colour& col) {
    drawLine(w, t.vertices[0], t.vertices[1], col);
    drawLine(w, t.vertices[1], t.vertices[2], col);
    drawLine(w, t.vertices[2], t.vertices[0], col);
}

static glm::vec3 rotateAroundY(const glm::vec3& v, float angle) {
    float c = std::cos(angle);
    float s = std::sin(angle);

    glm::mat3 Ry(
        glm::vec3( c, 0, -s),
        glm::vec3( 0, 1,  0),
        glm::vec3( s, 0,  c)
    );
    return Ry * v;
}

static glm::vec3 rotateAroundX(const glm::vec3& v, float angle) {
    float c = std::cos(angle);
    float s = std::sin(angle);

    glm::mat3 Rx(
        glm::vec3(1, 0,  0),
        glm::vec3(0,  c,  s),
        glm::vec3(0, -s,  c)
    );
    return Rx * v;
}

// cheap lookAt just for this coursework
static void lookAt(const glm::vec3& target){
    glm::vec3 forward = glm::normalize(target - g_camera);
    glm::vec3 worldUp(0.0f, 1.0f, 0.0f);

    glm::vec3 right = glm::normalize(glm::cross(forward, worldUp));
    if (glm::length(right) < 1e-4f) {
        worldUp = glm::vec3(0.0f, 0.0f, 1.0f);
        right   = glm::normalize(glm::cross(forward, worldUp));
    }

    glm::vec3 up = glm::normalize(glm::cross(right, forward));

    g_camOrientation = glm::mat3(
        right,
        up,
        -forward
    );
}

static inline CanvasPoint projectVertexOntoCanvasPoint(const glm::vec3& cameraPos,
    float focalLength,const glm::vec3& vertexPos,
    int W, int H,float imgScale
) {
    glm::vec3 rel = vertexPos - cameraPos;
    glm::vec3 cam = rel * g_camOrientation;

    float z = -cam.z;
    if (z <= 0.0f) return CanvasPoint(-1e9f, -1e9f);

    float u = (focalLength * cam.x / z) * imgScale + W * 0.5f;
    float v = (focalLength * -cam.y / z) * imgScale + H * 0.5f;

    CanvasPoint cp(u, v);
    cp.depth = 1.0f / z;
    return cp;
}

static glm::vec3 rayDirectionForPixel(int x, int y, int W, int H) {
    float px = x + 0.5f;
    float py = y + 0.5f;

    float camX = (px - W * 0.5f) / g_imgScale;
    float camY = -(py - H * 0.5f) / g_imgScale;
    float camZ = -g_focal;

    glm::vec3 dirCam(camX, camY, camZ);
    dirCam = glm::normalize(dirCam);

    glm::mat3 camToWorld = glm::transpose(g_camOrientation);
    glm::vec3 dirWorld   = dirCam * camToWorld;

    return glm::normalize(dirWorld);
}

// figure out floor bounds so we can tile texture
static void computeFloorBounds() {
    g_floorBoundsInit = false;
    if (g_model.empty()) return;

    bool first = true;

    for (const auto& mt : g_model) {
        if (!isFloorTriangleWorld(mt)) continue;

        for (int i = 0; i < 3; ++i) {
            const glm::vec3& v = mt.vertices[i];

            if (first) {
                g_floorMinX = g_floorMaxX = v.x;
                g_floorMinZ = g_floorMaxZ = v.z;
                first = false;
            } else {
                g_floorMinX = std::min(g_floorMinX, v.x);
                g_floorMaxX = std::max(g_floorMaxX, v.x);
                g_floorMinZ = std::min(g_floorMinZ, v.z);
                g_floorMaxZ = std::max(g_floorMaxZ, v.z);
            }
        }
    }
    g_floorBoundsInit = !first;
}

static TexturePoint floorUV(const glm::vec3& worldPos) {
    if (!g_floorBoundsInit) {
        return TexturePoint(0.0f, 0.0f);
    }

    float u = (worldPos.x - g_floorMinX) / (g_floorMaxX - g_floorMinX);
    float v = (worldPos.z - g_floorMinZ) / (g_floorMaxZ - g_floorMinZ);

    const float tiles = 1.9f;
    u *= tiles;
    v *= tiles;

    u -= std::floor(u);
    v -= std::floor(v);

    float texU = u * g_floorTex.width;
    float texV = v * g_floorTex.height;

    return TexturePoint(texU, texV);
}

// shading from ray tracer side
static uint32_t shadeHit(const RayTriangleIntersection& hit) {
    if (hit.triangleIndex < 0 ||
        hit.distanceFromCamera == std::numeric_limits<float>::infinity()) {
        return packARGB(255, 0, 0, 0);
    }

    const ModelTriangle& tri = hit.intersectedTriangle;
    glm::vec3 P = hit.intersectionPoint;

    const int   GRID        = SHADOW_GRID;
    const int   NUM_SAMPLES = GRID * GRID;

    const float ambientMin    = 0.20f;
    const float diffuseScale  = 1.0f;
    const float shadowAmbient = 0.10f;

    glm::vec3 toLightCentre = g_lightPos - P;
    float     lightDistC    = glm::length(toLightCentre);

    float brightnessFull = ambientMin;

    if (lightDistC > 0.0f) {
        float d       = std::max(lightDistC, 0.3f);
        float falloff = 1.0f / (4.0f * PI * d * d);
        float scale   = 12.0f;
        float proximity = glm::clamp(scale * falloff, 0.0f, 1.0f);

        glm::vec3 N = shadingNormal(tri, P);
        glm::vec3 L = glm::normalize(toLightCentre);

        float nDotL = glm::dot(N, L);
        if (nDotL < 0.0f) nDotL = 0.0f;
        float diffuse = proximity * nDotL;

        float specular = 0.0f;

        if (isFloorTriangleWorld(tri)) {
            glm::vec3 diff = P - g_lightPos;
            diff.y = 0.0f;

            float r2 = glm::dot(diff, diff);
            const float radius       = 0.55f;
            const float specStrength = 3.0f;

            float spot = std::exp(-r2 / (radius * radius));
            specular   = proximity * specStrength * spot;
        }
        else {
            glm::vec3 V = glm::normalize(g_camera - P);
            glm::vec3 H = glm::normalize(L + V);
            float nDotH = glm::dot(N, H);

            if (nDotH > 0.0f) {
                const float shininess    = 64.0f;
                const float specStrength = 1.0f;
                specular = proximity * specStrength * std::pow(nDotH, shininess);
            }
        }

        brightnessFull = ambientMin + diffuseScale * diffuse + specular;
    }
    brightnessFull = glm::clamp(brightnessFull, 0.0f, 1.0f);

    float visibleCount = 0.0f;

    for (int sy = 0; sy < GRID; ++sy) {
        for (int sx = 0; sx < GRID; ++sx) {
            float fx = (sx + 0.5f) / GRID - 0.5f;
            float fz = (sy + 0.5f) / GRID - 0.5f;

            float jx = fx * 2.0f * LIGHT_RADIUS;
            float jz = fz * 2.0f * LIGHT_RADIUS;

            glm::vec3 sampleLightPos = g_lightPos + glm::vec3(jx, 0.0f, jz);

            glm::vec3 toLight   = sampleLightPos - P;
            float     lightDist = glm::length(toLight);
            if (lightDist <= 0.0f) continue;

            glm::vec3 shadowDir  = glm::normalize(toLight);
            const float EPS      = 0.001f;
            glm::vec3 shadowOrig = P + shadowDir * EPS;

            RayTriangleIntersection shadowHit =
                getClosestIntersection(
                    shadowOrig,
                    shadowDir,
                    hit.triangleIndex,
                    lightDist - EPS
                );

            bool blocked = false;
            if (shadowHit.triangleIndex >= 0 &&
                shadowHit.distanceFromCamera > 0.0f &&
                shadowHit.distanceFromCamera < lightDist) {
                blocked = true;
            }

            if (!blocked) visibleCount += 1.0f;
        }
    }

    float visibility     = visibleCount / float(NUM_SAMPLES);
    float brightnessBase = shadowAmbient + visibility * (brightnessFull - shadowAmbient);

    float brightness = brightnessBase;

    glm::vec3 diff3 = P - g_lightPos;
    float r2 = glm::dot(diff3, diff3);

    float haloRadius   = 0.5f;
    float haloStrength = 1.0f;

    if (!g_isSphereScene) {
        haloRadius   = 0.70f;
        haloStrength = 3.0f;
    }

    float halo           = std::exp(-r2 / (haloRadius * haloRadius));
    float haloBrightness = ambientMin + haloStrength * halo * visibility;

    if (haloBrightness > brightness) brightness = haloBrightness;

    brightness = glm::clamp(brightness, 0.0f, 1.0f);

    float contrast = 1.3f;
    brightness = 0.5f + (brightness - 0.5f) * contrast;
    brightness = glm::clamp(brightness, 0.0f, 1.0f);

    if (!g_isSphereScene) {
        brightness *= CORNELL_RT_BRIGHTNESS;
        brightness = glm::clamp(brightness, 0.0f, 1.0f);
    }

    Colour c = tri.colour;
    c.red   = (int)std::round(c.red   * brightness);
    c.green = (int)std::round(c.green * brightness);
    c.blue  = (int)std::round(c.blue  * brightness);

    uint32_t argb = colourToARGB(c);
    return argb;
}

static void drawFilledTriangleZ(DrawingWindow& w,const CanvasTriangle& tri,const Colour& fill){
    const CanvasPoint& a = tri.vertices[0];
    const CanvasPoint& b = tri.vertices[1];
    const CanvasPoint& c = tri.vertices[2];

    uint32_t argb = colourToARGB(fill);

    float minX = std::min({ a.x, b.x, c.x });
    float maxX = std::max({ a.x, b.x, c.x });
    float minY = std::min({ a.y, b.y, c.y });
    float maxY = std::max({ a.y, b.y, c.y });

    int x0 = std::max(0, (int)std::floor(minX));
    int x1 = std::min((int)w.width  - 1, (int)std::ceil(maxX));
    int y0 = std::max(0, (int)std::floor(minY));
    int y1 = std::min((int)w.height - 1, (int)std::ceil(maxY));

    auto edge = [](const CanvasPoint& p0, const CanvasPoint& p1, float x, float y) {
        return (x - p0.x) * (p1.y - p0.y) - (y - p0.y) * (p1.x - p0.x);
    };

    float area = edge(a, b, c.x, c.y);
    if (area == 0.0f) return;
    float invArea = 1.0f / area;

    for (int y = y0; y <= y1; ++y) {
        for (int x = x0; x <= x1; ++x) {
            float px = x + 0.5f;
            float py = y + 0.5f;

            float w0 = edge(b, c, px, py);
            float w1 = edge(c, a, px, py);
            float w2 = edge(a, b, px, py);

            bool inside =
                (area > 0 && w0 >= 0 && w1 >= 0 && w2 >= 0) ||
                (area < 0 && w0 <= 0 && w1 <= 0 && w2 <= 0);
            if (!inside) continue;

            float l0 = w0 * invArea;
            float l1 = w1 * invArea;
            float l2 = w2 * invArea;

            float invZ = l0 * a.depth + l1 * b.depth + l2 * c.depth;
            putIfNearer(w, x, y, invZ, argb);
        }
    }
}

// textured tri with perspective correct uv  standard)
static void drawTexturedTriangleZ(DrawingWindow& w,const CanvasTriangle& tri,const TextureMap& tex)
{
    const CanvasPoint& a = tri.vertices[0];
    const CanvasPoint& b = tri.vertices[1];
    const CanvasPoint& c = tri.vertices[2];

    const TexturePoint& ta = a.texturePoint;
    const TexturePoint& tb = b.texturePoint;
    const TexturePoint& tc = c.texturePoint;

    float minX = std::min({ a.x, b.x, c.x });
    float maxX = std::max({ a.x, b.x, c.x });
    float minY = std::min({ a.y, b.y, c.y });
    float maxY = std::max({ a.y, b.y, c.y });

    int x0 = std::max(0, (int)std::floor(minX));
    int x1 = std::min((int)w.width  - 1, (int)std::ceil(maxX));
    int y0 = std::max(0, (int)std::floor(minY));
    int y1 = std::min((int)w.height - 1, (int)std::ceil(maxY));

    auto edge = [](const CanvasPoint& p0, const CanvasPoint& p1, float x, float y) {
        return (x - p0.x) * (p1.y - p0.y) - (y - p0.y) * (p1.x - p0.x);
    };

    float area = edge(a, b, c.x, c.y);
    if (area == 0.0f) return;
    float invArea = 1.0f / area;

    float az = a.depth;
    float bz = b.depth;
    float cz = c.depth;

    float auz = ta.x * az;
    float avz = ta.y * az;
    float buz = tb.x * bz;
    float bvz = tb.y * bz;
    float cuz = tc.x * cz;
    float cvz = tc.y * cz;

    for (int y = y0; y <= y1; ++y) {
        for (int x = x0; x <= x1; ++x) {
            float px = x + 0.5f;
            float py = y + 0.5f;

            float w0 = edge(b, c, px, py);
            float w1 = edge(c, a, px, py);
            float w2 = edge(a, b, px, py);

            bool inside =
                (area > 0 && w0 >= 0 && w1 >= 0 && w2 >= 0) ||
                (area < 0 && w0 <= 0 && w1 <= 0 && w2 <= 0);
            if (!inside) continue;

            float l0 = w0 * invArea;
            float l1 = w1 * invArea;
            float l2 = w2 * invArea;

            float invZ = l0 * az + l1 * bz + l2 * cz;
            if (invZ <= 0.0f) continue;

            float u_over_z = l0 * auz + l1 * buz + l2 * cuz;
            float v_over_z = l0 * avz + l1 * bvz + l2 * cvz;

            float u = u_over_z / invZ;
            float v = v_over_z / invZ;

            uint32_t argb = sampleTextureARGB(tex,
                                              (int)std::lround(u),
                                              (int)std::lround(v));

            putIfNearer(w, x, y, invZ, argb);
        }
    }
}

static bool projectTriangleToCanvas(const ModelTriangle& mt,int W, int H, CanvasTriangle& out){
    CanvasPoint a = projectVertexOntoCanvasPoint(g_camera, g_focal, mt.vertices[0], W, H, g_imgScale);
    CanvasPoint b = projectVertexOntoCanvasPoint(g_camera, g_focal, mt.vertices[1], W, H, g_imgScale);
    CanvasPoint c = projectVertexOntoCanvasPoint(g_camera, g_focal, mt.vertices[2], W, H, g_imgScale);

    if (a.x < -1e9f || b.x < -1e9f || c.x < -1e9f) return false;
    out = CanvasTriangle(a, b, c);
    return true;
}

// focus on center pixel (for physical dof)
static void autoFocusOnCenterPixel() {
    int cx = WIDTH  / 2;
    int cy = HEIGHT / 2;

    glm::vec3 dir = rayDirectionForPixel(cx, cy, WIDTH, HEIGHT);
    RayTriangleIntersection hit =
        getClosestIntersection(g_camera, dir);

    if (hit.triangleIndex >= 0 &&
        hit.distanceFromCamera < std::numeric_limits<float>::infinity()) {

        g_focusDistance = hit.distanceFromCamera;

        glm::vec3 rel = hit.intersectionPoint - g_camera;
        glm::vec3 cam = rel * g_camOrientation;
        g_focusDepthZ  = -cam.z;
    }
}

// blurs the colour buffer for raster dof, not physically perfect
static void applyRasterDOF(DrawingWindow& w) {
    if (!g_enableDOF || g_apertureRadius <= 0.0f)
        return;

    const int W = (int)w.width;
    const int H = (int)w.height;

    if ((int)g_colourBuffer.size() != W * H)
        return;

    std::vector<uint32_t> original = g_colourBuffer;
    std::vector<uint32_t> blurred(W * H);

    int   maxRadius = 8;
    int   RBlur     = (int)std::floor(g_apertureRadius * 20.0f);
    if (RBlur < 1) RBlur = 1;
    if (RBlur > maxRadius) RBlur = maxRadius;

    for (int y = 0; y < H; ++y) {
        for (int x = 0; x < W; ++x) {
            glm::vec3 accum(0.0f);
            int count = 0;

            for (int dy = -RBlur; dy <= RBlur; ++dy) {
                int yy = y + dy;
                if (yy < 0 || yy >= H) continue;

                for (int dx = -RBlur; dx <= RBlur; ++dx) {
                    int xx = x + dx;
                    if (xx < 0 || xx >= W) continue;
                    if (dx*dx + dy*dy > RBlur*RBlur) continue;

                    size_t sidx = (size_t)yy * W + xx;
                    uint32_t c  = original[sidx];

                    uint8_t r = (c >> 16) & 0xFF;
                    uint8_t g = (c >> 8)  & 0xFF;
                    uint8_t b =  c        & 0xFF;

                    accum += glm::vec3(r, g, b);
                    ++count;
                }
            }

            if (count > 0) {
                glm::vec3 avg = accum / float(count);
                uint8_t R8 = (uint8_t)std::round(avg.r);
                uint8_t G8 = (uint8_t)std::round(avg.g);
                uint8_t B8 = (uint8_t)std::round(avg.b);
                blurred[(size_t)y * W + x] = packARGB(255, R8, G8, B8);
            } else {
                blurred[(size_t)y * W + x] = original[(size_t)y * W + x];
            }
        }
    }

    if (g_dofMode == DOF_PHYSICAL) {
        float blurScale = g_apertureRadius * 20.0f;

        for (int y = 0; y < H; ++y) {
            for (int x = 0; x < W; ++x) {
                size_t idx = (size_t)y * W + x;
                float invZ = g_depth[idx];

                uint32_t out = blurred[idx];

                if (invZ > 0.0f) {
                    float z   = 1.0f / invZ;
                    float coc = blurScale * std::fabs(z - g_focusDepthZ);
                    if (coc < 0.5f) {
                        out = original[idx];
                    }
                }

                w.setPixelColour(x, y, out);
                g_colourBuffer[idx] = out;
            }
        }
        return;
    }

    if (g_dofMode == DOF_OBJECT_LOCK) {
        std::vector<uint8_t> mask(W * H, 0);

        for (int y = 0; y < H; ++y) {
            for (int x = 0; x < W; ++x) {
                glm::vec3 dir = rayDirectionForPixel(x, y, W, H);
                RayTriangleIntersection hit =
                    getClosestIntersection(g_camera, dir);

                if (hit.triangleIndex >= 0 &&
                    hit.distanceFromCamera < std::numeric_limits<float>::infinity() &&
                    hit.intersectedTriangle.colour.name == g_dofFocusMatName)
                {
                    mask[(size_t)y * W + x] = 1;
                }
            }
        }

        const int RFocus = 1;
        std::vector<uint8_t> maskDil = mask;

        for (int y = 0; y < H; ++y) {
            for (int x = 0; x < W; ++x) {
                if (!mask[(size_t)y * W + x]) continue;

                for (int dy = -RFocus; dy <= RFocus; ++dy) {
                    int yy = y + dy;
                    if (yy < 0 || yy >= H) continue;

                    for (int dx = -RFocus; dx <= RFocus; ++dx) {
                        int xx = x + dx;
                        if (xx < 0 || xx >= W) continue;
                        if (dx*dx + dy*dy > RFocus*RFocus) continue;

                        maskDil[(size_t)yy * W + xx] = 1;
                    }
                }
            }
        }

        for (int y = 0; y < H; ++y) {
            for (int x = 0; x < W; ++x) {
                size_t idx = (size_t)y * W + x;
                uint32_t out =
                    maskDil[idx] ? original[idx]
                                 : blurred[idx];

                w.setPixelColour(x, y, out);
                g_colourBuffer[idx] = out;
            }
        }
    }
}

// focus on all pixels of that material
static void focusOnMaterialFull(const std::string& targetMat) {
    float sumDist  = 0.0f;
    float minDist  = std::numeric_limits<float>::infinity();
    float maxDist  = 0.0f;
    int   hitCount = 0;

    for (int y = 0; y < HEIGHT; ++y) {
        for (int x = 0; x < WIDTH; ++x) {
            glm::vec3 dir = rayDirectionForPixel(x, y, WIDTH, HEIGHT);
            RayTriangleIntersection hit =
                getClosestIntersection(g_camera, dir);

            if (hit.triangleIndex >= 0 &&
                hit.distanceFromCamera < std::numeric_limits<float>::infinity() &&
                hit.intersectedTriangle.colour.name == targetMat)
            {
                float d = hit.distanceFromCamera;
                sumDist += d;
                minDist  = std::min(minDist, d);
                maxDist  = std::max(maxDist, d);
                ++hitCount;
            }
        }
    }

    if (hitCount > 0) {
        float mid = 0.5f * (minDist + maxDist);
        g_focusDistance = mid;
        g_focusDepthZ   = mid;
    } else {
        autoFocusOnCenterPixel();
    }
}

static const int MAX_RAY_DEPTH = 3;

// core ray tracer: refraction / metals / mirrors etc
static uint32_t traceRay(const glm::vec3& origin,const glm::vec3& direction,int depth){
    if (depth <= 0) {
        return packARGB(255, 0, 0, 0);
    }

    RayTriangleIntersection hit =
        getClosestIntersection(origin, direction);

    if (hit.triangleIndex < 0 ||
        hit.distanceFromCamera == std::numeric_limits<float>::infinity()) {
        return packARGB(255, 0, 0, 0);
    }

    const ModelTriangle& tri = hit.intersectedTriangle;
    glm::vec3 P = hit.intersectionPoint;

    glm::vec3 D = glm::normalize(direction);
    glm::vec3 N = shadingNormal(tri, P);

    if (isRefractiveTriangle(tri)) {
        const float n_air   = 1.0f;
        const float n_glass = 1.5f;

        float n1, n2;
        float cosTheta1;
        glm::vec3 Nf = N;

        float dDotN = glm::dot(D, N);

        if (dDotN < 0.0f) {
            n1 = n_air;
            n2 = n_glass;
            cosTheta1 = -dDotN;
        }
        else {
            n1 = n_glass;
            n2 = n_air;
            Nf = -N;
            cosTheta1 = -glm::dot(D, Nf);
        }

        float eta = n1 / n2;
        float k   = 1.0f - eta * eta * (1.0f - cosTheta1 * cosTheta1);

        glm::vec3 reflectDir =
            glm::normalize(D - 2.0f * glm::dot(D, Nf) * Nf);
        const float EPS = 0.001f;
        glm::vec3 reflectOrigin = P + reflectDir * EPS;

        if (k < 0.0f) {
            return traceRay(reflectOrigin, reflectDir, depth - 1);
        }

        glm::vec3 refractDir =
            glm::normalize(eta * D + (eta * cosTheta1 - std::sqrt(k)) * Nf);
        glm::vec3 refractOrigin = P + refractDir * EPS;

        float R0 = (n1 - n2) / (n1 + n2);
        R0 = R0 * R0;
        float Rf = R0 + (1.0f - R0) * std::pow(1.0f - cosTheta1, 5.0f);
        Rf = glm::clamp(Rf, 0.0f, 1.0f);

        uint32_t reflARGB = traceRay(reflectOrigin, reflectDir, depth - 1);
        uint32_t refrARGB = traceRay(refractOrigin, refractDir, depth - 1);

        Colour cr = argbToColour(reflARGB);
        Colour ct = argbToColour(refrARGB);

        float rr = cr.red   / 255.0f;
        float rg = cr.green / 255.0f;
        float rb = cr.blue  / 255.0f;

        float tr = ct.red   / 255.0f;
        float tg = ct.green / 255.0f;
        float tb = ct.blue  / 255.0f;

        float fr = Rf * rr + (1.0f - Rf) * tr;
        float fg = Rf * rg + (1.0f - Rf) * tg;
        float fb = Rf * rb + (1.0f - Rf) * tb;

        fr = glm::clamp(fr, 0.0f, 1.0f);
        fg = glm::clamp(fg, 0.0f, 1.0f);
        fb = glm::clamp(fb, 0.0f, 1.0f);

        return packARGB(
            255,
            (uint8_t)std::round(fr * 255.0f),
            (uint8_t)std::round(fg * 255.0f),
            (uint8_t)std::round(fb * 255.0f)
        );
    }
    else if (isIronTriangle(tri)) {
        const int   IRON_SAMPLES = 5;
        const float IRON_ROUGH   = 0.22f;

        glm::vec3 accumRGB(0.0f);
        int validSamples = 0;

        for (int s = 0; s < IRON_SAMPLES; ++s) {
            glm::vec3 R = glm::normalize(D - 2.0f * glm::dot(D, N) * N);
            glm::vec3 Rp = perturbDirection(R, IRON_ROUGH);

            const float EPS = 0.001f;
            glm::vec3 reflOrigin = P + Rp * EPS;

            uint32_t sampleARGB = traceRay(reflOrigin, Rp, depth - 1);
            Colour c = argbToColour(sampleARGB);

            accumRGB += glm::vec3(
                c.red   / 255.0f,
                c.green / 255.0f,
                c.blue  / 255.0f
            );
            ++validSamples;
        }

        if (validSamples == 0) {
            return packARGB(255, 0, 0, 0);
        }

        glm::vec3 avgRGB = accumRGB / float(validSamples);
        glm::vec3 ironTint(0.68f, 0.70f, 0.73f);

        glm::vec3 finalRGB = avgRGB * ironTint;
        finalRGB = glm::clamp(finalRGB, glm::vec3(0.0f), glm::vec3(1.0f));

        return packARGB(
            255,
            (uint8_t)std::round(finalRGB.r * 255.0f),
            (uint8_t)std::round(finalRGB.g * 255.0f),
            (uint8_t)std::round(finalRGB.b * 255.0f)
        );
    }
    else if (isMetalTriangle(tri)) {
        const int   METAL_SAMPLES = 4;
        const float METAL_ROUGH   = 0.08f;

        glm::vec3 accumRGB(0.0f);
        int validSamples = 0;

        for (int s = 0; s < METAL_SAMPLES; ++s) {
            glm::vec3 R  = glm::normalize(D - 2.0f * glm::dot(D, N) * N);
            glm::vec3 Rp = perturbDirection(R, METAL_ROUGH);
            const float EPS = 0.001f;
            glm::vec3 reflOrigin = P + Rp * EPS;

            uint32_t sampleARGB = traceRay(reflOrigin, Rp, depth - 1);
            Colour c = argbToColour(sampleARGB);

            accumRGB += glm::vec3(
                c.red   / 255.0f,
                c.green / 255.0f,
                c.blue  / 255.0f
            );
            ++validSamples;
        }

        if (validSamples == 0) {
            return packARGB(255, 0, 0, 0);
        }

        glm::vec3 avgRGB = accumRGB / float(validSamples);

        glm::vec3 metalTint = GOLD_TINT;

        glm::vec3 finalRGB = avgRGB * metalTint;
        finalRGB = glm::clamp(finalRGB, glm::vec3(0.0f), glm::vec3(1.0f));

        return packARGB(
            255,
            (uint8_t)std::round(finalRGB.r * 255.0f),
            (uint8_t)std::round(finalRGB.g * 255.0f),
            (uint8_t)std::round(finalRGB.b * 255.0f)
        );
    }

    if (isMirrorTriangle(tri)) {
        glm::vec3 R = glm::normalize(D - 2.0f * glm::dot(D, N) * N);
        const float EPS = 0.001f;
        glm::vec3 reflOrigin = P + R * EPS;
        return traceRay(reflOrigin, R, depth - 1);
    }

    return shadeHit(hit);
}

// multi threaded raytrace, splits by rows
static void drawRayTracedRange(DrawingWindow& w, int yStart, int yEnd) {
    const int W = (int)w.width;
    const int H = (int)w.height;

    glm::mat3 camToWorld = glm::transpose(g_camOrientation);

    bool useDOFPhysical =
        g_enableDOF &&
        g_dofMode == DOF_PHYSICAL &&
        g_dofSamples > 1 &&
        g_apertureRadius > 0.0f &&
        g_focusDistance > 0.0f;

    for (int y = yStart; y < yEnd; ++y) {
        for (int x = 0; x < W; ++x) {

            glm::vec3 baseDir = rayDirectionForPixel(x, y, W, H);

            if (!useDOFPhysical) {
                uint32_t argb = traceRay(g_camera, baseDir, MAX_RAY_DEPTH);
                putColour(w, x, y, argb);
                continue;
            }

            glm::vec3 focusPoint = g_camera + g_focusDistance * baseDir;

            glm::vec3 accumRGB(0.0f);
            int nSamples = g_dofSamples;

            for (int s = 0; s < nSamples; ++s) {
                glm::vec3 offsetCam   = sampleApertureOffsetCam(g_apertureRadius);
                glm::vec3 offsetWorld = offsetCam * camToWorld;
                glm::vec3 origin      = g_camera + offsetWorld;
                glm::vec3 dir         = glm::normalize(focusPoint - origin);

                uint32_t sampleARGB = traceRay(origin, dir, MAX_RAY_DEPTH);
                Colour c = argbToColour(sampleARGB);

                accumRGB += glm::vec3(
                    c.red   / 255.0f,
                    c.green / 255.0f,
                    c.blue  / 255.0f
                );
            }

            glm::vec3 avgRGB = accumRGB / float(nSamples);
            avgRGB = glm::clamp(avgRGB, glm::vec3(0.0f), glm::vec3(1.0f));

            uint8_t R = (uint8_t)std::round(avgRGB.r * 255.0f);
            uint8_t G = (uint8_t)std::round(avgRGB.g * 255.0f);
            uint8_t B = (uint8_t)std::round(avgRGB.b * 255.0f);

            uint32_t argb = packARGB(255, R, G, B);
            putColour(w, x, y, argb);
        }
    }
}

static void drawRayTraced(DrawingWindow& w) {
    const int H = (int)w.height;

    if (g_enableDOF) {
        if (g_dofMode == DOF_PHYSICAL) {
            autoFocusOnCenterPixel();
        }
        else if (g_dofMode == DOF_OBJECT_LOCK) {
            focusOnMaterialFull(g_dofFocusMatName);
        }
    }

    int nThreads = g_numThreads;

    if (nThreads <= 0) {
        unsigned int hw = std::thread::hardware_concurrency();
        if (hw == 0) hw = 8;
        nThreads = (int)hw;
    }

    if (nThreads < 1)  nThreads = 1;
    if (nThreads > H)  nThreads = H;

    std::vector<std::thread> threads;
    threads.reserve(nThreads);

    int rowsPerThread = H / nThreads;
    int yStart = 0;

    for (int i = 0; i < nThreads; ++i) {
        int yEnd = (i == nThreads - 1) ? H : (yStart + rowsPerThread);
        threads.emplace_back(drawRayTracedRange, std::ref(w), yStart, yEnd);
        yStart = yEnd;
    }

    for (auto& t : threads) {
        t.join();
    }

    if (g_enableDOF && g_dofMode == DOF_OBJECT_LOCK) {
        applyRasterDOF(w);
    }
}

static void drawWireframe(DrawingWindow& w) {
    const int W = (int)w.width;
    Colour white(255, 255, 255);

    for (const auto& mt : g_model) {
        CanvasTriangle ct;
        if (!projectTriangleToCanvas(mt, W, (int)w.height, ct)) continue;
        drawStrokedTriangle(w, ct, white);
    }
}

// env map sampling
static uint32_t sampleEnvARGB(const glm::vec3& dirWorld) {
    if (g_envMap.width == 0 || g_envMap.height == 0) {
        return packARGB(255, 128, 128, 128);
    }

    glm::vec3 d = glm::normalize(dirWorld);

    float u = std::atan2(d.z, d.x) / (2.0f * PI) + 0.5f;
    float v = 0.5f - std::asin(d.y) / PI;

    int texU = (int)std::floor(u * g_envMap.width);
    int texV = (int)std::floor(v * g_envMap.height);

    return sampleTextureARGB(g_envMap, texU, texV);
}

// env mapped sphere, reflect into panorama
static void drawEnvSphereTriangleZ(DrawingWindow& w,
                                   const CanvasTriangle& tri,
                                   const glm::vec3& P0,
                                   const glm::vec3& P1,
                                   const glm::vec3& P2)
{
    const CanvasPoint& a = tri.vertices[0];
    const CanvasPoint& b = tri.vertices[1];
    const CanvasPoint& c = tri.vertices[2];

    float minX = std::min({ a.x, b.x, c.x });
    float maxX = std::max({ a.x, b.x, c.x });
    float minY = std::min({ a.y, b.y, c.y });
    float maxY = std::max({ a.y, b.y, c.y });

    int x0 = std::max(0, (int)std::floor(minX));
    int x1 = std::min((int)w.width  - 1, (int)std::ceil(maxX));
    int y0 = std::max(0, (int)std::floor(minY));
    int y1 = std::min((int)w.height - 1, (int)std::ceil(maxY));

    auto edge = [](const CanvasPoint& p0, const CanvasPoint& p1, float x, float y) {
        return (x - p0.x) * (p1.y - p0.y) - (y - p0.y) * (p1.x - p0.x);
    };

    float area = edge(a, b, c.x, c.y);
    if (area == 0.0f) return;
    float invArea = 1.0f / area;

    for (int y = y0; y <= y1; ++y) {
        for (int x = x0; x <= x1; ++x) {
            float px = x + 0.5f;
            float py = y + 0.5f;

            float w0 = edge(b, c, px, py);
            float w1 = edge(c, a, px, py);
            float w2 = edge(a, b, px, py);

            bool inside =
                (area > 0 && w0 >= 0 && w1 >= 0 && w2 >= 0) ||
                (area < 0 && w0 <= 0 && w1 <= 0 && w2 <= 0);
            if (!inside) continue;

            float l0 = w0 * invArea;
            float l1 = w1 * invArea;
            float l2 = w2 * invArea;

            float invZ = l0 * a.depth + l1 * b.depth + l2 * c.depth;

            glm::vec3 P = l0 * P0 + l1 * P1 + l2 * P2;

            glm::vec3 N = glm::normalize(P - g_sphereCentre);
            glm::vec3 I = glm::normalize(P - g_camera);
            glm::vec3 R = glm::reflect(I, N);

            uint32_t argb = sampleEnvARGB(R);

            putIfNearer(w, x, y, invZ, argb);
        }
    }
}

// background from env map for that sphere scene
static void drawEnvBackground(DrawingWindow& w) {
    const int W = (int)w.width;
    const int H = (int)w.height;

    for (int y = 0; y < H; ++y) {
        for (int x = 0; x < W; ++x) {
            glm::vec3 dir = rayDirectionForPixel(x, y, W, H);
            uint32_t argb = sampleEnvARGB(dir);
            putColour(w, x, y, argb);
            g_depth[(size_t)y * W + x] = 0.0f;
        }
    }
}

// main rasteriser pass, with texture and gouraud
static void drawRasterised(DrawingWindow& w) {
    const int W = (int)w.width;
    const int H = (int)w.height;

    glm::vec3 oldLightPos = g_lightPos;

    if (g_enableDOF) {
        if (g_dofMode == DOF_PHYSICAL) {
            autoFocusOnCenterPixel();
        }
        else if (g_dofMode == DOF_OBJECT_LOCK) {
            focusOnMaterialFull(g_dofFocusMatName);
        }
    }

    if (g_sceneType == SCENE_ENV_SPHERE) {
        drawEnvBackground(w);

        for (const auto& mt : g_model) {
            glm::vec3 P0 = mt.vertices[0];
            glm::vec3 P1 = mt.vertices[1];
            glm::vec3 P2 = mt.vertices[2];

            CanvasPoint a = projectVertexOntoCanvasPoint(g_camera, g_focal, P0, W, H, g_imgScale);
            CanvasPoint b = projectVertexOntoCanvasPoint(g_camera, g_focal, P1, W, H, g_imgScale);
            CanvasPoint c = projectVertexOntoCanvasPoint(g_camera, g_focal, P2, W, H, g_imgScale);

            if (a.x < -1e9f || b.x < -1e9f || c.x < -1e9f)
                continue;

            CanvasTriangle ct(a, b, c);
            drawEnvSphereTriangleZ(w, ct, P0, P1, P2);
        }

        applyRasterDOF(w);

        g_lightPos = oldLightPos;
        return;
    }

    if (g_isSphereScene) {
        for (const auto& mt : g_model) {
            glm::vec3 P0 = mt.vertices[0];
            glm::vec3 P1 = mt.vertices[1];
            glm::vec3 P2 = mt.vertices[2];

            CanvasPoint a = projectVertexOntoCanvasPoint(g_camera, g_focal, P0, W, H, g_imgScale);
            CanvasPoint b = projectVertexOntoCanvasPoint(g_camera, g_focal, P1, W, H, g_imgScale);
            CanvasPoint c = projectVertexOntoCanvasPoint(g_camera, g_focal, P2, W, H, g_imgScale);

            if (a.x < -1e9f || b.x < -1e9f || c.x < -1e9f)
                continue;

            CanvasTriangle ct(a, b, c);

            glm::vec3 N0 = glm::normalize(P0 - g_sphereCentre);
            glm::vec3 N1 = glm::normalize(P1 - g_sphereCentre);
            glm::vec3 N2 = glm::normalize(P2 - g_sphereCentre);

            drawPhongTriangleZ(w, ct, P0, P1, P2, N0, N1, N2, mt.colour);
        }

        applyRasterDOF(w);

        g_lightPos = oldLightPos;
        return;
    }

    for (const auto& mt : g_model) {
        if (isFloorTriangleWorld(mt)) {
            CanvasTriangle ct;
            if (!projectTriangleToCanvas(mt, W, H, ct)) continue;

            ct.vertices[0].texturePoint = floorUV(mt.vertices[0]);
            ct.vertices[1].texturePoint = floorUV(mt.vertices[1]);
            ct.vertices[2].texturePoint = floorUV(mt.vertices[2]);

            drawTexturedTriangleZ(w, ct, g_floorTex);
            continue;
        }

        glm::vec3 P0 = mt.vertices[0];
        glm::vec3 P1 = mt.vertices[1];
        glm::vec3 P2 = mt.vertices[2];

        CanvasPoint a = projectVertexOntoCanvasPoint(g_camera, g_focal, P0, W, H, g_imgScale);
        CanvasPoint b = projectVertexOntoCanvasPoint(g_camera, g_focal, P1, W, H, g_imgScale);
        CanvasPoint c = projectVertexOntoCanvasPoint(g_camera, g_focal, P2, W, H, g_imgScale);

        if (a.x < -1e9f || b.x < -1e9f || c.x < -1e9f)
            continue;

        CanvasTriangle ct(a, b, c);

        glm::vec3 N0 = shadingNormal(mt, P0);
        glm::vec3 N1 = shadingNormal(mt, P1);
        glm::vec3 N2 = shadingNormal(mt, P2);

        ct.vertices[0].brightness = computeVertexBrightness(P0, N0);
        ct.vertices[1].brightness = computeVertexBrightness(P1, N1);
        ct.vertices[2].brightness = computeVertexBrightness(P2, N2);

        drawGouraudTriangleZ(w, ct, mt.colour);
    }

    applyRasterDOF(w);

    g_lightPos = oldLightPos;
}

// main draw entry used by framework
void draw(DrawingWindow& window) {
    window.clearPixels();

    if (g_model.empty()) return;

    if (g_orbit) {
        static int s_orbitCounter = 0;

        int skip = g_playIdent ? 3 : 1;

        bool moveThisFrame = true;
        if (++s_orbitCounter < skip) {
            moveThisFrame = false;
        } else {
            s_orbitCounter = 0;
        }

        if (moveThisFrame) {
            const float orbitStep = 0.01f * skip;

            glm::vec3 centre = g_isSphereScene ? g_sphereCentre : g_sceneCentre;

            glm::vec3 rel = g_camera - centre;
            if (glm::length(rel) < 0.001f) {
                rel = glm::vec3(0.0f, 0.0f, 4.0f);
            }

            rel      = rotateAroundY(rel, orbitStep);
            g_camera = centre + rel;

            lookAt(centre);

            if (g_isSphereScene) {
                float a = g_sphereBaseTiltRad;
                float c = std::cos(a), s = std::sin(a);
                glm::mat3 Rx(
                    glm::vec3(1, 0,  0),
                    glm::vec3(0,  c,  s),
                    glm::vec3(0, -s,  c)
                );
                g_camOrientation = g_camOrientation * Rx;
            }
        }
    }

    int W = (int)window.width;
    int H = (int)window.height;

    if ((int)g_depth.size() != W * H) {
        g_depth.assign(W * H, 0.0f);
    } else {
        std::fill(g_depth.begin(), g_depth.end(), 0.0f);
    }

    if ((int)g_colourBuffer.size() != W * H) {
        g_colourBuffer.assign(W * H, 0u);
    } else {
        std::fill(g_colourBuffer.begin(), g_colourBuffer.end(), 0u);
    }

    switch (g_mode) {
    case MODE_NONE:
        break;
    case MODE_WIREFRAME:
        drawWireframe(window);
        break;
    case MODE_RASTERISED:
        drawRasterised(window);
        break;
    case MODE_RAYTRACED:
        drawRayTraced(window);
        break;
    }
}

// mtl loader, quite basic
static std::unordered_map<std::string, Colour> loadMTL(const std::string& mtlPath) {
    std::ifstream in(mtlPath);
    std::unordered_map<std::string, Colour> palette;

    std::string line, current;
    while (std::getline(in, line)) {
        size_t p = line.find_first_not_of(" \t\r");
        if (p == std::string::npos) continue;
        if (line[p] == '#') continue;

        if (line.compare(p, 7, "newmtl ") == 0) {
            current = line.substr(p + 7);
            while (!current.empty() && isspace((unsigned char)current.back()))
                current.pop_back();
        }
        else if (line.compare(p, 3, "Kd ") == 0 && !current.empty()) {
            std::istringstream ss(line.substr(p + 3));
            float r, g, b;
            if (ss >> r >> g >> b) {
                int R = (int)std::lround(std::max(0.f, std::min(1.f, r)) * 255.f);
                int G = (int)std::lround(std::max(0.f, std::min(1.f, g)) * 255.f);
                int B = (int)std::lround(std::max(0.f, std::min(1.f, b)) * 255.f);
                Colour c(R, G, B);
                c.name = current;
                palette[current] = c;
            }
        }
    }

    std::cout << "Loaded " << palette.size() << " materials from " << mtlPath << "\n";
    return palette;
}

// obj loader that triangulates faces, uses usemtl
static std::vector<ModelTriangle> loadOBJ(const std::string& objPath,
                                          float scale,
                                          const std::unordered_map<std::string, Colour>& palette)
{
    std::ifstream in(objPath);
    if (!in) {
        std::cerr << "ERROR: couldn't open OBJ: " << objPath << "\n";
        return {};
    }

    auto idx0 = [](const std::string& tok) -> int {
        size_t slash = tok.find('/');
        const std::string head = (slash == std::string::npos) ? tok : tok.substr(0, slash);
        try {
            return std::stoi(head) - 1;
        } catch (...) {
            return -999999;
        }
    };

    std::vector<glm::vec3> verts;
    std::vector<ModelTriangle> tris;
    std::string currentMtl;

    std::string line;
    while (std::getline(in, line)) {
        size_t p = line.find_first_not_of(" \t\r");
        if (p == std::string::npos) continue;
        if (line[p] == '#') continue;

        if (line.compare(p, 2, "v ") == 0) {
            std::istringstream ss(line.substr(p + 2));
            float x, y, z;
            if (ss >> x >> y >> z) {
                verts.emplace_back(x * scale, y * scale, z * scale);
            }
        }
        else if (line.compare(p, 2, "f ") == 0) {
            std::istringstream ss(line.substr(p + 2));
            std::vector<std::string> toks;
            std::string tok;
            while (ss >> tok) toks.push_back(tok);

            if (toks.size() >= 3) {
                auto it = palette.find(currentMtl);
                Colour faceCol = (it != palette.end()) ? it->second : Colour(255, 255, 255);

                for (size_t i = 1; i + 1 < toks.size(); ++i) {
                    int i0 = idx0(toks[0]);
                    int i1 = idx0(toks[i]);
                    int i2 = idx0(toks[i + 1]);

                    if (i0 < 0 || i1 < 0 || i2 < 0) continue;
                    if (i0 >= (int)verts.size() || i1 >= (int)verts.size() || i2 >= (int)verts.size()) continue;

                    ModelTriangle tri;
                    tri.vertices[0] = verts[i0];
                    tri.vertices[1] = verts[i1];
                    tri.vertices[2] = verts[i2];
                    tri.colour      = faceCol;

                    glm::vec3 e0 = tri.vertices[1] - tri.vertices[0];
                    glm::vec3 e1 = tri.vertices[2] - tri.vertices[0];

                    glm::vec3 n = glm::normalize(glm::cross(e0, e1));

                    glm::vec3 triCentre = (tri.vertices[0] +
                                           tri.vertices[1] +
                                           tri.vertices[2]) / 3.0f;
                    glm::vec3 toCentre  = g_sceneCentre - triCentre;

                    if (glm::dot(n, toCentre) < 0.0f) {
                        n = -n;
                    }

                    tri.normal = n;

                    tris.push_back(tri);
                }
            }
        }
        else if (line.compare(p, 7, "usemtl ") == 0) {
            currentMtl = line.substr(p + 7);
            while (!currentMtl.empty() && isspace((unsigned char)currentMtl.back()))
                currentMtl.pop_back();
        }
    }
    return tris;
}

// compute center-ish for model
static glm::vec3 computeModelCentre(const std::vector<ModelTriangle>& model) {
    if (model.empty()) return glm::vec3(0.0f);

    glm::vec3 minV(
        std::numeric_limits<float>::infinity(),
        std::numeric_limits<float>::infinity(),
        std::numeric_limits<float>::infinity()
    );
    glm::vec3 maxV(
        -std::numeric_limits<float>::infinity(),
        -std::numeric_limits<float>::infinity(),
        -std::numeric_limits<float>::infinity()
    );

    for (const auto& tri : model) {
        for (int i = 0; i < 3; ++i) {
            const glm::vec3& v = tri.vertices[i];
            minV.x = std::min(minV.x, v.x);
            minV.y = std::min(minV.y, v.y);
            minV.z = std::min(minV.z, v.z);
            maxV.x = std::max(maxV.x, v.x);
            maxV.y = std::max(maxV.y, v.y);
            maxV.z = std::max(maxV.z, v.z);
        }
    }

    return 0.5f * (minV + maxV);
}

// set camera for sphere default shot
static void setSphereDefaultView() {
    g_camera = g_sphereCamPos;
    lookAt(g_sphereCentre);

    float a = g_sphereBaseTiltRad;
    float c = std::cos(a), s = std::sin(a);
    glm::mat3 Rx(
        glm::vec3(1, 0,  0),
        glm::vec3(0,  c,  s),
        glm::vec3(0, -s,  c)
    );
    g_camOrientation = g_camOrientation * Rx;
}

// sphere scene: just red sphere
static void initSphereScene() {
    g_palette.clear();

    std::string spherePath =
        "sphere.obj";

    g_model = loadOBJ(spherePath, 0.6f, g_palette);

    for (auto& tri : g_model) {
        tri.colour      = Colour(220, 60, 60);
        tri.colour.name = "RedSphere";
    }

    g_sphereCentre = computeModelCentre(g_model);
    g_sceneCentre  = g_sphereCentre;

    g_isSphereScene = true;

    setSphereDefaultView();

    g_lightPos = glm::vec3(5.0f, 9.0f, -9.0f);
}

// env sphere is same sphere but reflective
static void initEnvSphereScene() {
    g_palette.clear();

    std::string spherePath =
        "sphere.obj";

    g_model = loadOBJ(spherePath, 0.6f, g_palette);

    for (auto& tri : g_model) {
        tri.colour      = Colour(220, 60, 60);
        tri.colour.name = "EnvSphere";
    }

    g_sphereCentre = computeModelCentre(g_model);
    g_sceneCentre  = g_sphereCentre;

    g_isSphereScene = true;
    setSphereDefaultView();

    g_lightPos = glm::vec3(5.0f, 9.0f, -9.0f);
}

// cornell box init
static void initCornellScene() {
    g_palette.clear();

    std::string mtlPath =
        "cornell-box.mtl";
    std::string objPath =
        "cornell-box.obj";

    g_palette = loadMTL(mtlPath);
    g_model   = loadOBJ(objPath, 0.6f, g_palette);

    g_sceneCentre = computeModelCentre(g_model);

    g_isSphereScene = false;

    g_camera = glm::vec3(0.0f, 0.0f, -4.0f);
    lookAt(g_sceneCentre);

    g_lightPos = glm::vec3(0.0f, 1.5f, 1.0f);

    computeFloorBounds();
}

// helper to make sure correct scene loaded
static void ensureScene(SceneType desired) {
    if (g_sceneType == desired && !g_model.empty()) return;

    g_sceneType = desired;

    switch (desired) {
        case SCENE_CORNELL:
            initCornellScene();
            break;

        case SCENE_SPHERE:
            initSphereScene();
            break;

        case SCENE_ENV_SPHERE:
            initEnvSphereScene();
            break;
    }
}

// just clamps 0..1 into steps
static float quantize01(float u, int steps) {
    if (steps <= 1) return 0.0f;
    u = glm::clamp(u, 0.0f, 1.0f);
    float step = 1.0f / float(steps - 1);
    return std::round(u / step) * step;
}

// little helper curve for ident cornell arc
static glm::vec3 cornellArcOffset(float u, const glm::vec3& cornellCentre) {
    glm::vec3 baseCam    = glm::vec3(0.0f, 1.0f, 12.0f);
    glm::vec3 baseOffset = baseCam - cornellCentre;

    const float yaw = 0.55f;

    glm::vec3 rightOffset = rotateAroundY(baseOffset, +yaw);
    glm::vec3 leftOffset  = rotateAroundY(baseOffset, -yaw);

    glm::vec3 midOffset = 0.5f * (rightOffset + leftOffset);
    midOffset.y += 3.0f;

    glm::vec3 offset;

    if (u < 0.2f) {
        float v = u / 0.2f;
        offset = (1.0f - v) * baseOffset + v * rightOffset;
    }
    else if (u < 0.8f) {
        float v = (u - 0.2f) / 0.6f;

        float w0 = (1.0f - v) * (1.0f - v);
        float w1 = 2.0f * (1.0f - v) * v;
        float w2 = v * v;

        offset = w0 * rightOffset + w1 * midOffset + w2 * leftOffset;
    }
    else {
        float v = (u - 0.8f) / 0.2f;
        offset = (1.0f - v) * leftOffset + v * baseOffset;
    }

    return offset;
}

// this is the big ident animation state machine, bit messy but does job
static void updateIdentAnimation() {
    if (!g_playIdent) return;

    const float IDENT_SPEED = 0.3f;
    const float MAX_DT      = 1.0f / 60.0f;

    const float PHASE0_END  =  1.0f;
    const float PHASE1_END  =  2.0f;
    const float PHASE2_END  =  5.0f;
    const float PHASE3_END  =  8.0f;
    const float PHASE4_END  = 11.0f;
    const float PHASE5_END  = 14.0f;
    const float PHASE6_END  = 17.0f;
    const float PHASE7_END  = 20.0f;
    const float PHASE8_END  = 23.0f;
    const float PHASE9_END  = 26.0f;
    const float PHASE10_END = 29.0f;
    const float PHASE11_END = 32.0f;

    static bool   s_timeInit  = false;
    static float  s_identTime = 0.0f;
    static Uint32 s_lastTicks = 0;

    Uint32 now = SDL_GetTicks();

    if (!s_timeInit) {
        s_timeInit  = true;
        s_lastTicks = now;
        s_identTime = 0.0f;
    }

    float dt = (now - s_lastTicks) * 0.001f;
    s_lastTicks = now;
    if (dt > MAX_DT) dt = MAX_DT;

    s_identTime += dt * IDENT_SPEED;
    float t = s_identTime;

    glm::vec3 cornellCentre  = g_sceneCentre;
    glm::vec3 cornellBaseCam = glm::vec3(0.0f, 1.0f, 12.0f);

    auto doCornellArc = [&](float tStart, float tEnd) {
        glm::vec3 baseOffset = cornellBaseCam - cornellCentre;
        const float yaw      = 0.55f;

        glm::vec3 rightOffset = rotateAroundY(baseOffset, +yaw);
        glm::vec3 leftOffset  = rotateAroundY(baseOffset, -yaw);
        glm::vec3 midOffset   = 0.5f * (rightOffset + leftOffset);
        midOffset.y += 3.0f;

        float u = (t - tStart) / (tEnd - tStart);
        u = glm::clamp(u, 0.0f, 1.0f);

        glm::vec3 offset;
        if (u < 0.2f) {
            float v = u / 0.2f;
            offset = (1.0f - v) * baseOffset + v * rightOffset;
        }
        else if (u < 0.8f) {
            float v  = (u - 0.2f) / 0.6f;
            float w0 = (1.0f - v) * (1.0f - v);
            float w1 = 2.0f * (1.0f - v) * v;
            float w2 = v * v;
            offset   = w0 * rightOffset + w1 * midOffset + w2 * leftOffset;
        }
        else {
            float v = (u - 0.8f) / 0.2f;
            offset  = (1.0f - v) * leftOffset + v * baseOffset;
        }

        g_camera = cornellCentre + offset;
        lookAt(cornellCentre);
    };

    if (t < PHASE0_END) {
        g_mode      = MODE_NONE;
        g_orbit     = false;
        g_enableDOF = false;
        return;
    }
    else if (t < PHASE1_END) {
        ensureScene(SCENE_CORNELL);
        g_isSphereScene = false;
        g_mode          = MODE_WIREFRAME;
        g_orbit         = false;
        g_enableDOF     = false;

        g_camera = cornellBaseCam;
        lookAt(cornellCentre);
    }
    else if (t < PHASE2_END) {
        ensureScene(SCENE_CORNELL);
        g_isSphereScene = false;
        g_mode          = MODE_RASTERISED;
        g_orbit         = false;
        g_enableDOF     = false;

        float u = (t - PHASE1_END) / (PHASE2_END - PHASE1_END);
        u = glm::clamp(u, 0.0f, 1.0f);

        const int NUM_STEPS = 9;
        int step = (int)std::floor(u * NUM_STEPS);
        if (step >= NUM_STEPS) step = NUM_STEPS - 1;

        glm::vec3 offset(0.0f);
        switch (step) {
            case 0: offset = glm::vec3(0.0f,   0.0f, 0.0f);   break;
            case 1: offset = glm::vec3(-0.6f,  0.0f, 0.0f);   break;
            case 2: offset = glm::vec3(-1.2f,  0.0f, 0.0f);   break;
            case 3: offset = glm::vec3(+0.6f,  0.0f, 0.0f);   break;
            case 4: offset = glm::vec3(+1.2f,  0.0f, 0.0f);   break;
            case 5: offset = glm::vec3(0.0f,  +0.6f, 0.0f);   break;
            case 6: offset = glm::vec3(0.0f,  +1.2f, 0.0f);   break;
            case 7: offset = glm::vec3(0.0f,  -0.6f, 0.0f);   break;
            case 8: offset = glm::vec3(0.0f,  -1.2f, 0.0f);   break;
        }

        g_camera = cornellBaseCam + offset;
        lookAt(cornellCentre);
    }
    else if (t < PHASE3_END) {
        ensureScene(SCENE_CORNELL);
        g_isSphereScene = false;
        g_mode          = MODE_RASTERISED;
        g_orbit         = false;
        g_enableDOF     = false;

        g_camera = cornellBaseCam;
        lookAt(cornellCentre);
        glm::mat3 baseOrient = g_camOrientation;

        float u = (t - PHASE2_END) / (PHASE3_END - PHASE2_END);
        u = quantize01(u, 20);

        const int NUM_SEGS = 4;
        float segLen = 1.0f / NUM_SEGS;
        int   seg    = (int)std::floor(u / segLen);
        if (seg >= NUM_SEGS) seg = NUM_SEGS - 1;
        float v = (u - seg * segLen) / segLen;

        float totalAngle = 2.0f * CAM_ROT_STEP;
        float angle      = totalAngle * v;

        auto makeYaw = [](float a) {
            float c = std::cos(a), s = std::sin(a);
            return glm::mat3(
                glm::vec3( c, 0, -s),
                glm::vec3( 0, 1,  0),
                glm::vec3( s, 0,  c)
            );
        };
        auto makePitch = [](float a) {
            float c = std::cos(a), s = std::sin(a);
            return glm::mat3(
                glm::vec3(1, 0,  0),
                glm::vec3(0,  c,  s),
                glm::vec3(0, -s,  c)
            );
        };

        glm::mat3 R(1.0f);
        switch (seg) {
            case 0: R = makeYaw(+angle); break;
            case 1: R = makeYaw(-angle); break;
            case 2: R = makePitch(+angle); break;
            case 3: R = makePitch(-angle); break;
        }

        g_camOrientation = baseOrient * R;
    }
    else if (t < PHASE4_END) {
        ensureScene(SCENE_CORNELL);
        g_isSphereScene = false;
        g_mode          = MODE_RASTERISED;
        g_enableDOF     = false;

        static bool s_cornellHorizInit = false;
        if (!s_cornellHorizInit) {
            g_camera = cornellBaseCam;
            lookAt(cornellCentre);
            s_cornellHorizInit = true;
        }

        g_orbit = true;
    }
    else if (t < PHASE5_END) {
        ensureScene(SCENE_CORNELL);
        g_isSphereScene = false;
        g_mode          = MODE_RASTERISED;
        g_orbit         = false;
        g_enableDOF     = false;

        float u = (t - PHASE4_END) / (PHASE5_END - PHASE4_END);
        u = quantize01(u, 24);

        float radius = 12.0f;
        float angleX = u * 2.0f * PI;

        glm::vec3 offset(0.0f, 0.0f, radius);
        offset = rotateAroundX(offset, angleX);

        g_camera = cornellCentre + offset;
        lookAt(cornellCentre);
    }
    else if (t < PHASE6_END) {
        ensureScene(SCENE_SPHERE);
        g_isSphereScene = true;
        g_mode          = MODE_RASTERISED;
        g_orbit         = true;
        g_enableDOF     = false;
    }
    else if (t < PHASE7_END) {
        ensureScene(SCENE_ENV_SPHERE);
        g_isSphereScene = true;
        g_mode          = MODE_RASTERISED;
        g_orbit         = true;
        g_enableDOF     = false;
    }
    else if (t < PHASE8_END) {
        ensureScene(SCENE_CORNELL);

        g_isSphereScene = false;
        g_mode          = MODE_RAYTRACED;
        g_orbit         = false;
        g_enableDOF     = false;
        g_dofMode       = DOF_OFF;
        g_identMatMode  = IDENT_MAT_NORMAL;

        doCornellArc(PHASE7_END, PHASE8_END);
    }
    else if (t < PHASE9_END) {
        ensureScene(SCENE_CORNELL);

        g_isSphereScene = false;
        g_mode          = MODE_RAYTRACED;
        g_orbit         = false;
        g_identMatMode  = IDENT_MAT_NORMAL;

        float u = (t - PHASE8_END) / (PHASE9_END - PHASE8_END);
        u = glm::clamp(u, 0.0f, 1.0f);

        int seg;
        if (u < 1.0f / 3.0f)      seg = 0;
        else if (u < 2.0f / 3.0f) seg = 1;
        else                      seg = 2;

        static int s_prevSeg = -1;
        if (seg != s_prevSeg) {
            s_prevSeg = seg;

            if (seg == 0) {
                setSoftBlur();
                g_dofMode         = DOF_OBJECT_LOCK;
                g_dofFocusMatName = "Blue";
                g_enableDOF       = true;
            }
            else if (seg == 1) {
                setSoftBlur();
                g_dofMode         = DOF_OBJECT_LOCK;
                g_dofFocusMatName = "Red";
                g_enableDOF       = true;
            }
            else {
                g_enableDOF = false;
                g_dofMode   = DOF_OFF;
            }
        }
    }
    else if (t < PHASE10_END) {
        ensureScene(SCENE_CORNELL);

        g_isSphereScene = false;
        g_mode          = MODE_RAYTRACED;
        g_orbit         = false;
        g_enableDOF     = false;
        g_dofMode       = DOF_OFF;
        g_identMatMode  = IDENT_MAT_MIRROR_GLASS;

        doCornellArc(PHASE9_END, PHASE10_END);
    }
    else if (t < PHASE11_END) {
        ensureScene(SCENE_CORNELL);

        g_isSphereScene = false;
        g_mode          = MODE_RAYTRACED;
        g_orbit         = false;
        g_enableDOF     = false;
        g_dofMode       = DOF_OFF;
        g_identMatMode  = IDENT_MAT_IRON_GOLD;

        doCornellArc(PHASE10_END, PHASE11_END);
    }
    else {
        g_identMatMode = IDENT_MAT_OFF;
        g_playIdent    = false;
    }
}

static int g_frameSkip = 30; 

static void saveFrame(DrawingWindow &window) {
    char filename[256];
    std::snprintf(
        filename,
        sizeof(filename),
        "src/frames/frame_%04d.bmp",
        g_frameIndex
    );

    window.saveBMP(filename);
    std::cout << "saved frame " << g_frameIndex
              << " -> " << filename << "\n";
    g_frameIndex++;
}

// event handler, keys and mouse, bit long but ok
void handleEvent(SDL_Event event, DrawingWindow& window) {
    if (event.type == SDL_KEYDOWN) {
        if (event.key.keysym.sym == SDLK_ESCAPE) {
            std::exit(0);
        }
        else if (event.key.keysym.sym == SDLK_w) {
            g_mode = MODE_WIREFRAME;
        }
        else if (event.key.keysym.sym == SDLK_r) {
            g_mode  = MODE_RASTERISED;
            g_orbit = false;

            if (g_sceneType == SCENE_SPHERE) {
                setSphereDefaultView();
            } else {
                g_camera = glm::vec3(0.0f, 0.0f, 11.0f);
                lookAt(g_sceneCentre);
            }
        }
        else if (event.key.keysym.sym == SDLK_t) {
            g_mode = MODE_RAYTRACED;
        }

        else if (event.key.keysym.sym == SDLK_LEFT) {
            g_camera.x -= CAM_MOVE_STEP;
        }
        else if (event.key.keysym.sym == SDLK_RIGHT) {
            g_camera.x += CAM_MOVE_STEP;
        }
        else if (event.key.keysym.sym == SDLK_UP) {
            g_camera.y += CAM_MOVE_STEP;
        }
        else if (event.key.keysym.sym == SDLK_DOWN) {
            g_camera.y -= CAM_MOVE_STEP;
        }
        else if (event.key.keysym.sym == SDLK_HOME) {
            g_camera.z -= CAM_MOVE_STEP;
        }
        else if (event.key.keysym.sym == SDLK_END) {
            g_camera.z += CAM_MOVE_STEP;
        }

        else if (event.key.keysym.sym == SDLK_7) {
            g_lightPos.y += LIGHT_MOVE_STEP;
        }
        else if (event.key.keysym.sym == SDLK_6) {
            g_lightPos.y -= LIGHT_MOVE_STEP;
        }
        else if (event.key.keysym.sym == SDLK_8) {
            g_lightPos.x -= LIGHT_MOVE_STEP;
        }
        else if (event.key.keysym.sym == SDLK_9) {
            g_lightPos.x += LIGHT_MOVE_STEP;
        }
        else if (event.key.keysym.sym == SDLK_0) {
            g_lightPos.z += LIGHT_MOVE_STEP;
        }

        else if (event.key.keysym.sym == SDLK_LEFTBRACKET) {
            g_focusDistance *= 0.9f;
            g_focusDepthZ   *= 0.9f;
        }
        else if (event.key.keysym.sym == SDLK_RIGHTBRACKET) {
            g_focusDistance *= 1.1f;
            g_focusDepthZ   *= 1.1f;
        }

        else if (event.key.keysym.sym == SDLK_a) {
            glm::vec3 centre = g_isSphereScene ? g_sphereCentre : g_sceneCentre;
            glm::vec3 offset = g_camera - centre;
            offset = rotateAroundY(offset, +CAM_ROT_STEP);
            g_camera = centre + offset;
            lookAt(centre);
        }
        else if (event.key.keysym.sym == SDLK_d) {
            glm::vec3 centre = g_isSphereScene ? g_sphereCentre : g_sceneCentre;
            glm::vec3 offset = g_camera - centre;
            offset = rotateAroundY(offset, -CAM_ROT_STEP);
            g_camera = centre + offset;
            lookAt(centre);
        }
        else if (event.key.keysym.sym == SDLK_z) {
            glm::vec3 centre = g_isSphereScene ? g_sphereCentre : g_sceneCentre;
            glm::vec3 offset = g_camera - centre;
            offset = rotateAroundX(offset, +CAM_ROT_STEP);
            g_camera = centre + offset;
            lookAt(centre);
        }
        else if (event.key.keysym.sym == SDLK_x) {
            glm::vec3 centre = g_isSphereScene ? g_sphereCentre : g_sceneCentre;
            glm::vec3 offset = g_camera - centre;
            offset = rotateAroundX(offset, -CAM_ROT_STEP);
            g_camera = centre + offset;
            lookAt(centre);
        }

        else if (event.key.keysym.sym == SDLK_1) {
            g_dofMode   = DOF_OFF;
            g_enableDOF = false;
        }
        else if (event.key.keysym.sym == SDLK_2) {
            g_dofMode   = DOF_PHYSICAL;
            g_enableDOF = true;
            setSoftBlur();
        }
        else if (event.key.keysym.sym == SDLK_3) {
            g_dofMode   = DOF_OBJECT_LOCK;
            g_enableDOF = true;
            setSoftBlur();
        }

        else if (event.key.keysym.sym == SDLK_j) {
            float a = CAM_ROT_STEP;
            float c = std::cos(a), s = std::sin(a);
            glm::mat3 Ry(
                glm::vec3( c, 0, -s),
                glm::vec3( 0, 1,  0),
                glm::vec3( s, 0,  c)
            );
            g_camOrientation = g_camOrientation * Ry;
        }
        else if (event.key.keysym.sym == SDLK_l) {
            float a = -CAM_ROT_STEP;
            float c = std::cos(a), s = std::sin(a);
            glm::mat3 Ry(
                glm::vec3( c, 0, -s),
                glm::vec3( 0, 1,  0),
                glm::vec3( s, 0,  c)
            );
            g_camOrientation = g_camOrientation * Ry;
        }
        else if (event.key.keysym.sym == SDLK_i) {
            float a = CAM_ROT_STEP;
            float c = std::cos(a), s = std::sin(a);
            glm::mat3 Rx(
                glm::vec3(1, 0,  0),
                glm::vec3(0,  c,  s),
                glm::vec3(0, -s,  c)
            );
            g_camOrientation = g_camOrientation * Rx;
        }
        else if (event.key.keysym.sym == SDLK_k) {
            float a = -CAM_ROT_STEP;
            float c = std::cos(a), s = std::sin(a);
            glm::mat3 Rx(
                glm::vec3(1, 0,  0),
                glm::vec3(0,  c,  s),
                glm::vec3(0, -s,  c)
            );
            g_camOrientation = g_camOrientation * Rx;
        }

        else if (event.key.keysym.sym == SDLK_SPACE) {
            g_orbit = !g_orbit;
            if (g_orbit && g_isSphereScene) {
                setSphereDefaultView();
            }
        }
    }
    else if (event.type == SDL_MOUSEBUTTONDOWN) {
        if (event.button.button == SDL_BUTTON_LEFT) {
            int x = event.button.x;
            int y = event.button.y;

            glm::vec3 dir = rayDirectionForPixel(x, y, WIDTH, HEIGHT);
            RayTriangleIntersection hit =
                getClosestIntersection(g_camera, dir);

            if (hit.triangleIndex >= 0 &&
                hit.distanceFromCamera < std::numeric_limits<float>::infinity())
            {
                g_dofMode         = DOF_OBJECT_LOCK;
                g_dofFocusMatName = hit.intersectedTriangle.colour.name;
                g_enableDOF       = true;

                g_focusDistance = hit.distanceFromCamera;

                glm::vec3 rel = hit.intersectionPoint - g_camera;
                glm::vec3 cam = rel * g_camOrientation;
                g_focusDepthZ  = -cam.z;
            }
        }
    }
}

static const int THREADS_RESERVED_FOR_OS = 2;

// Guess how many threads to use for raytracing
static void initRayTracingThreads() {
    unsigned int hw = std::thread::hardware_concurrency();
    if (hw == 0) hw = 8;

    int n = (int)hw - THREADS_RESERVED_FOR_OS;

    if (n < 1)           n = 1;
    if (n > (int)hw - 1) n = (int)hw - 1;

    g_numThreads = n;
}

int main(int argc, char* argv[]) {
    DrawingWindow window = DrawingWindow(WIDTH, HEIGHT, false);
    std::srand(static_cast<unsigned>(std::time(nullptr)));

    initRayTracingThreads();

    if (g_sceneType == SCENE_SPHERE) {
        initSphereScene();
    } else if (g_sceneType == SCENE_CORNELL) {
        initCornellScene();
    } else if (g_sceneType == SCENE_ENV_SPHERE) {
        initEnvSphereScene();
    }

    g_mode      = MODE_RASTERISED;
    g_orbit     = false;
    g_enableDOF = false;
    g_dofMode   = DOF_OFF;

    SDL_Event event;

    while (true) {
        if (window.pollForInputEvents(event)) {
            handleEvent(event, window);
        }

        // move camera etc if ident animation
        if (g_playIdent) {
            updateIdentAnimation();
        }

        // draw one frame and show it
        static int frameCounter = 0;

        draw(window);
        window.renderFrame();

        if (g_recordFrames) {
            frameCounter++;

            // only save when frameCounter is a multiple of g_frameSkip
            if (frameCounter % g_frameSkip == 0) {
                saveFrame(window);
            }
        }

    }

    return 0;
}
