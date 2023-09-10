#include <jni.h>
#include <string.h>
#include <math.h>
#include <android/log.h>
#include <vector>

using namespace std;

#define LOG_TAG "JNI.LOG"
#define LOGD(...) __android_log_print(ANDROID_LOG_DEBUG, LOG_TAG, __VA_ARGS__)
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO, LOG_TAG, __VA_ARGS__)

static float DepthZ, CenterX, CenterY, CenterZ, Center_x, Center_y;
static float HPI, W2PI, W0, R, r, FirstR;

static int width, height, width00, height00, parent_width, parent_height;
static double factorA = 0, factorB = 0, factorC = 0, factorD = 0;
const double EPSILON = 0.0000000001;
static jbyte *BGBufOut;
static jbyte* CubeBuf[6];
static jbyte *CubeBufOut;
static jbyte* BallBuf;
static jbyte* FatherBuf;
static jbyte* MotherBuf;
static jbyte *BallBufOut;
static jbyteArray pCubeOutData, pBallOutData, pAllBGData;
static jbyteArray pCubeSrcData1, pCubeSrcData2, pCubeSrcData3
, pCubeSrcData4, pCubeSrcData5, pCubeSrcData6, pBallSrcData, pFatherSrcData, pMotherSrcData;
static float sx, sy, parent_r;
static int BGWidth, BGHeight2;

// 2D vector
struct Vector2d {
public:
    Vector2d() {
    }

    ~Vector2d() {
    }

    Vector2d(double dx, double dy) {
        x = dx;
        y = dy;
    }

    // 矢量赋值
    void set(double dx, double dy) {
        x = dx;
        y = dy;
    }

    // 矢量相加
    Vector2d operator+(const Vector2d &v) const {
        return Vector2d(x + v.x, y + v.y);
    }

    // 矢量相减
    Vector2d operator-(const Vector2d &v) const {
        return Vector2d(x - v.x, y - v.y);
    }

    //矢量数乘
    Vector2d Scalar(double c) const {
        return Vector2d(c * x, c * y);
    }

    // 矢量点积
    double Dot(const Vector2d &v) const {
        return x * v.x + y * v.y;
    }

    bool operator==(const Vector2d &v) const {
        if (abs(x - v.x) < EPSILON && abs(y - v.y) < EPSILON) {
            return true;
        }
        return false;
    }

    double x, y;
};

// 3D vector
struct Vector3d {
public:
    Vector3d() {
    }

    ~Vector3d() {
    }

    Vector3d(double dx, double dy, double dz) {
        x = dx;
        y = dy;
        z = dz;
    }

    // 矢量赋值
    void set(double dx, double dy, double dz) {
        x = dx;
        y = dy;
        z = dz;
    }

    // 矢量相加
    Vector3d operator+(const Vector3d &v) const {
        return Vector3d(x + v.x, y + v.y, z + v.z);
    }

    // 矢量相减
    Vector3d operator-(const Vector3d &v) const {
        return Vector3d(x - v.x, y - v.y, z - v.z);
    }

    //矢量数乘
    Vector3d Scalar(double c) const {
        return Vector3d(c * x, c * y, c * z);
    }

    // 矢量点积
    double Dot(const Vector3d &v) const {
        return x * v.x + y * v.y + z * v.z;
    }

    // 矢量叉积
    Vector3d Cross(const Vector3d &v) const {
        return Vector3d(y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x);
    }

    bool operator==(const Vector3d &v) const {
        if (abs(x - v.x) < EPSILON && abs(y - v.y) < EPSILON && abs(z - v.z) < EPSILON) {
            return true;
        }
        return false;
    }

    double x, y, z;
};

static Vector3d BallCenter, EYE;
static Vector3d myMap[1280][1280];
static Vector2d FacePoints[3][4];

//求解一元二次方程组ax*x + b*x + c = 0
void SolvingQuadratics(double a, double b, double c, vector<double> &t) {
    double delta = b * b - 4 * a * c;

    if (delta < 0) {
        return;
    }

    if (abs(delta) < EPSILON) {
        t.push_back(-b / (2 * a));
    } else {
        t.push_back((-b + sqrt(delta)) / (2 * a));
        t.push_back((-b - sqrt(delta)) / (2 * a));
    }
}

void LineIntersectSphere(Vector3d &E, double R, vector<Vector3d> &points) {
    Vector3d D = E - EYE;            //线段方向向量

    double a = (D.x * D.x) + (D.y * D.y) + (D.z * D.z);
    double b = (2 * D.x * (EYE.x - BallCenter.x) + 2 * D.y * (EYE.y - BallCenter.y) +
                2 * D.z * (EYE.z - BallCenter.z));
    double c = ((EYE.x - BallCenter.x) * (EYE.x - BallCenter.x) + (EYE.y - BallCenter.y) * (EYE.y - BallCenter.y) +
                (EYE.z - BallCenter.z) * (EYE.z - BallCenter.z)) - R * R;

    vector<double> t;
    SolvingQuadratics(a, b, c, t);

    for (auto it : t) {
        points.push_back(EYE + D.Scalar(it));
    }
}

double getDistance2(float x1, float y1, float x2, float y2) {
    return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

double getDistance32(Vector3d &P1, Vector3d &P2) {
    double ret = (P1.x - P2.x) * (P1.x - P2.x) + (P1.y - P2.y) * (P1.y - P2.y) +
                 +(P1.z - P2.z) * (P1.z - P2.z);
    return ret;
}

Vector3d GetFootOfPerpendicular(const Vector3d &p0, const Vector3d &p1, const Vector3d &p2) {
    double dx = p2.x - p1.x;
    double dy = p2.y - p1.y;
    double dz = p2.z - p1.z;

    double k = ((p1.x - p0.x) * dx + (p1.y - p0.y) * dy + (p1.z - p0.z) * dz)
               / ((dx * dx) + (dy * dy) + (dz * dz)) * -1;

    Vector3d retVal(k * dx + p1.x, k * dy + p1.y, k * dz + p1.z);

    return retVal;
}

void
getPanel(Vector3d &p1, Vector3d &p2, Vector3d &p3, double &a, double &b, double &c, double &d) {
    a = ((p2.y - p1.y) * (p3.z - p1.z) - (p2.z - p1.z) * (p3.y - p1.y));
    b = ((p2.z - p1.z) * (p3.x - p1.x) - (p2.x - p1.x) * (p3.z - p1.z));
    c = ((p2.x - p1.x) * (p3.y - p1.y) - (p2.y - p1.y) * (p3.x - p1.x));
    d = (0 - (a * p1.x + b * p1.y + c * p1.z));
}

void
getBallShowCenter() {
    Center_x = width / 2;
    Center_y = height / 2;        //if the 2D Ball center and screen center is same point.
    //Center_y = (EYE.z - DepthZ) * ((BallCenter.y - EYE.y) / (EYE.z - BallCenter.z));
}

bool isBack(Vector3d &p) {
    return factorA * p.x + factorB * p.y + factorC * p.z + factorD > 0;
}

double getLongitude(Vector3d &P, Vector3d &A, Vector3d &M) {
    //点P 到 北极，球心连线的 垂足
    Vector3d foot = GetFootOfPerpendicular(P, A, BallCenter);
    Vector3d a = P - foot;            // 垂足到点P的方向向量
    Vector3d b = M - BallCenter;        // 本初子午线与赤道的交点与地心的方向向量

    double c = (a.x * b.x + a.y * b.y + a.z * b.z) / sqrt(a.x * a.x + a.y * a.y + a.z * a.z)
               / sqrt(b.x * b.x + b.y * b.y + b.z * b.z);

    if (isBack(P)) {
        return 2 * 3.14159f - acos(c);
    }
    return acos(c);
}

extern "C"
JNIEXPORT void JNICALL Java_com_frank_family_BallLayout_InitializationBall(JNIEnv *env, jobject obj,
                                                                             const jfloat Z,
                                                                             const jfloat cx,
                                                                             const jfloat cy,
                                                                             const jfloat cz) {
    DepthZ = Z;
    CenterX = cx;
    CenterY = cy;
    CenterZ = cz;
    BallCenter.set(CenterX, CenterY, CenterZ);
}

extern "C"
JNIEXPORT void JNICALL Java_com_frank_family_BallLayout_InitializationBall2(JNIEnv *env, jobject obj,
                                                                              const jint width0,
                                                                              const jint height0,
                                                                              const jfloat R0,
                                                                              const jfloat r0,
                                                                              const jbyteArray pSrcData,
                                                                              const jint widthP,
                                                                              const jint heightP,
                                                                              const jbyteArray pSrcDataF,
                                                                              const jbyteArray pSrcDataM,
                                                                              const jbyteArray pOutData) {
    HPI = (double) height0 / 3.14159f;
    W0 = width0;
    W2PI = (double) width0 / 3.14159f / 2.0f;
    R = R0, r = r0, FirstR = r;
    getBallShowCenter();
    parent_width = widthP;
    parent_height = heightP;
    parent_r = parent_width / 2;

    for (int y = 0; y < height; y++) {
        if (y < Center_y - r || y > Center_y + r) {
            continue;
        }
        for (int x = 0; x < width; x++) {
            if (getDistance2(x, y, Center_x, Center_y) < r) {
                Vector3d E(x, y, DepthZ);
                vector<Vector3d> points;
                LineIntersectSphere(E, R, points);
                for (auto it : points) {
                    if (it.z >= CenterZ) {
                        Vector3d P(it.x, it.y, it.z);
                        myMap[y][x] = P;
                    }
                }
            }
        }
    }
    BallBuf = env->GetByteArrayElements(pSrcData, 0);
    FatherBuf = env->GetByteArrayElements(pSrcDataF, 0);
    MotherBuf = env->GetByteArrayElements(pSrcDataM, 0);
    BallBufOut = env->GetByteArrayElements(pOutData, 0);

    pBallSrcData = pSrcData;
    pFatherSrcData = pSrcDataF;
    pMotherSrcData = pSrcDataM;
    pBallOutData = pOutData;
}

extern "C"
JNIEXPORT jint JNICALL Java_com_frank_family_BallLayout_transformsBall(JNIEnv *env, jobject obj,
                                                                         const jfloat ArcticX,
                                                                         const jfloat ArcticY,
                                                                         const jfloat ArcticZ,
                                                                         const jfloat MeridianX,
                                                                         const jfloat MeridianY,
                                                                         const jfloat MeridianZ,
                                                                         const jfloat r0) {
    r = r0;
    double scale = r / FirstR;
    Vector3d Arctic(ArcticX, ArcticY, ArcticZ);
    Vector3d Meridian(MeridianX, MeridianY, MeridianZ);

    getPanel(Arctic, Meridian, BallCenter, factorA, factorB, factorC, factorD);

    double latitude, longitude;
    int original_point, pixel_point, x0, y0;

    float alpha1 = 3.14159f / 6.0f;
    float alpha2 = 3.14159f * 5 / 6.0f;
    for (int y = 0; y < height; y++) {
        if (y < Center_y - r || y > Center_y + r) {
            for (int x = 0; x < width; x++) {
                pixel_point = (width * y + x) * 4;
                int x_index = round(x / sx);
                int y_index = round(y / sy) + BGHeight2;
                original_point = (y_index * BGWidth + x_index) * 4;
                *(BallBufOut + pixel_point) = *(BGBufOut + original_point);
                *(BallBufOut + pixel_point + 1) = *(BGBufOut + original_point + 1);
                *(BallBufOut + pixel_point + 2) = *(BGBufOut + original_point + 2);
                *(BallBufOut + pixel_point + 3) = -1;
            }
            continue;
        }
        for (int x = 0; x < width; x++) {
            pixel_point = (width * y + x) * 4;
            if (getDistance2(x, y, Center_x, Center_y) < r) {
                Vector3d P = myMap[y][x];
                if (r == FirstR) {
                    P = myMap[y][x];
                } else {
                    x0 = (int) ((double) width / 2.0f +
                                ((double) x - (double) width / 2.0f) / scale);
                    y0 = (int) ((double) height / 2.0f +
                                ((double) y - (double) height / 2.0f) / scale);
                    if (x0 < 0) x0 = 0;
                    if (x0 >= width) x0 = width - 1;
                    if (y0 < 0) y0 = 0;
                    if (y0 >= height) y0 = height - 1;
                    P = myMap[y0][x0];
                }
                latitude = acos(1 - getDistance32(P, Arctic) / (2 * R * R));
                longitude = getLongitude(P, Arctic, Meridian);
                if (latitude < alpha1) {
int x_index = (round)(parent_r - cos(longitude + 3.1416) * parent_r * (latitude / alpha1));
int y_index = (round)(parent_r - sin(longitude + 3.1416) * parent_r * (latitude / alpha1));
                    original_point = (y_index * parent_width + x_index) * 4;
                    *(BallBufOut + pixel_point) = *(FatherBuf + original_point);
                    *(BallBufOut + pixel_point + 1) = *(FatherBuf + original_point + 1);
                    *(BallBufOut + pixel_point + 2) = *(FatherBuf + original_point + 2);
                    *(BallBufOut + pixel_point + 3) = -1;
                } else if (latitude > alpha2) {
int x_index = (round)(parent_r - cos(longitude) * parent_r * ((3.1416 - latitude) / (3.1416 - alpha2)));
int y_index = (round)(parent_r - sin(longitude) * parent_r * ((3.1416 - latitude) / (3.1416 - alpha2)));
                    original_point = (y_index * parent_width + x_index) * 4;
                    *(BallBufOut + pixel_point) = *(MotherBuf + original_point);
                    *(BallBufOut + pixel_point + 1) = *(MotherBuf + original_point + 1);
                    *(BallBufOut + pixel_point + 2) = *(MotherBuf + original_point + 2);
                    *(BallBufOut + pixel_point + 3) = -1;
                } else {
                    original_point = ((int) (HPI * latitude) * W0 + (int) (W2PI * longitude)) * 4;
                    *(BallBufOut + pixel_point) = *(BallBuf + original_point);
                    *(BallBufOut + pixel_point + 1) = *(BallBuf + original_point + 1);
                    *(BallBufOut + pixel_point + 2) = *(BallBuf + original_point + 2);
                    *(BallBufOut + pixel_point + 3) = -1;
                }
            } else {
                int x_index = round(x / sx);
                int y_index = round(y / sy) + BGHeight2;
                original_point = (y_index * BGWidth + x_index) * 4;
                *(BallBufOut + pixel_point) = *(BGBufOut + original_point);
                *(BallBufOut + pixel_point + 1) = *(BGBufOut + original_point + 1);
                *(BallBufOut + pixel_point + 2) = *(BGBufOut + original_point + 2);
                *(BallBufOut + pixel_point + 3) = -1;
            }
        }
    }
//    HPI = (double) height0 / 3.14159f;
//    W0 = width0;
//    W2PI = (double) width0 / 3.14159f / 2.0f;
//    R = R0, r = r0, FirstR = r;

    return 0;
}

extern "C"
JNIEXPORT void JNICALL Java_com_frank_family_MainActivity_setEYE(JNIEnv *env, jclass obj,
                                                                 const jint w, const jint h,
                                                                 const jfloat e_x, const jfloat e_y, const jfloat e_z,
                                                                 const jint bgW, const jint bgH,
                                                                 const jbyteArray pBGData) {
    width = w;
    height = h;
    EYE.set(e_x, e_y, e_z);
    sx = (float)width / (float)bgW;
    sy = (float)height / ((float)bgH / 2);
    BGWidth = bgW;
    BGHeight2 = bgH / 2;
    pAllBGData = pBGData;
    BGBufOut = env->GetByteArrayElements(pBGData, 0);
}

extern "C"
JNIEXPORT jint JNICALL Java_com_frank_family_CubeLayout_isCubeVisible(JNIEnv *env, jclass obj,
                                                              const jfloat xE,
                                                              const jfloat yE,
                                                              const jfloat zE,
                                                              const jfloat xF,
                                                              const jfloat yF,
                                                              const jfloat zF,
                                                              const jfloat xG,
                                                              const jfloat yG,
                                                              const jfloat zG) {
    Vector3d E(xE, yE, zE);
    Vector3d F(xF, yF, zF);
    Vector3d G(xG, yG, zG);

    Vector3d EF = F - E;
    Vector3d EG = G - E;
    Vector3d N = EF.Cross(EG);
    Vector3d V = EYE - E;

    if (N.Dot(V) > 0) {
        return 0;
    } else {
        return 1;
    }
}

extern "C"
JNIEXPORT void JNICALL
Java_com_frank_family_CubeLayout_InitializationCube(JNIEnv *env, jobject obj,
                                                      const jint w00,
                                                      const jint h00,
                                                      const jbyteArray pSrcData1,
                                                      const jbyteArray pSrcData2,
                                                      const jbyteArray pSrcData3,
                                                      const jbyteArray pSrcData4,
                                                      const jbyteArray pSrcData5,
                                                      const jbyteArray pSrcData6,
                                                      const jbyteArray pOutData) {
    width00 = w00;
    height00 = h00;
    CubeBuf[0] = env->GetByteArrayElements(pSrcData1, 0);
    CubeBuf[1] = env->GetByteArrayElements(pSrcData2, 0);
    CubeBuf[2] = env->GetByteArrayElements(pSrcData3, 0);
    CubeBuf[3] = env->GetByteArrayElements(pSrcData4, 0);
    CubeBuf[4] = env->GetByteArrayElements(pSrcData5, 0);
    CubeBuf[5] = env->GetByteArrayElements(pSrcData6, 0);
    CubeBufOut = env->GetByteArrayElements(pOutData, 0);

    pCubeSrcData1 = pSrcData1;
    pCubeSrcData2 = pSrcData2;
    pCubeSrcData3 = pSrcData3;
    pCubeSrcData4 = pSrcData4;
    pCubeSrcData5 = pSrcData5;
    pCubeSrcData6 = pSrcData6;
    pCubeOutData = pOutData;
}

struct quat {
    Vector2d points[4];
};

float cross(Vector2d a, Vector2d b) { return a.x * b.y - a.y * b.x; }

bool inquat(int index, float x, float y) {
    Vector2d v1, v2;
    v1.x = FacePoints[index][1].x - FacePoints[index][0].x;
    v1.y = FacePoints[index][1].y- FacePoints[index][0].y;
    v2.x = x - FacePoints[index][0].x;
    v2.y = y - FacePoints[index][0].y;
    if (cross(v2, v1) < 0) {
        return false;
    }
    v1.x = FacePoints[index][2].x - FacePoints[index][1].x;
    v1.y = FacePoints[index][2].y- FacePoints[index][1].y;
    v2.x = x - FacePoints[index][1].x;
    v2.y = y - FacePoints[index][1].y;
    if (cross(v2, v1) < 0) {
        return false;
    }
    v1.x = FacePoints[index][3].x - FacePoints[index][2].x;
    v1.y = FacePoints[index][3].y- FacePoints[index][2].y;
    v2.x = x - FacePoints[index][2].x;
    v2.y = y - FacePoints[index][2].y;
    if (cross(v2, v1) < 0) {
        return false;
    }
    v1.x = FacePoints[index][0].x - FacePoints[index][3].x;
    v1.y = FacePoints[index][0].y- FacePoints[index][3].y;
    v2.x = x - FacePoints[index][3].x;
    v2.y = y - FacePoints[index][3].y;
    if (cross(v2, v1) < 0) {
        return false;
    }
    return true;
}

Vector2d invBilinear(Vector2d p, int index) {

    Vector2d a = FacePoints[index][1];
    Vector2d b = FacePoints[index][0];
    Vector2d c = FacePoints[index][3];
    Vector2d d = FacePoints[index][2];

    Vector2d e = b - a;
    Vector2d f = d - a;
    Vector2d g = a - b + c - d;
    Vector2d h = p - a;

    float k2 = cross(g, f);
    float k1 = cross(e, f) + cross(h, g);
    float k0 = cross(h, e);

    if( abs(k2) < 0.001 )
    {
        return Vector2d( (h.x*k1+f.x*k0)/(e.x*k1-g.x*k0), -k0/k1 );
    }

    float w = k1 * k1 - 4.0 * k0 * k2;
    if (w < 0.0) return Vector2d(-1, 0);

    w = sqrt(w);

    float v = (-k1 - w) / (2.0 * k2);
    float u = (h.x - f.x * v) / (e.x + g.x * v);

    if (v < 0.0 || v > 1.0 || u < 0.0 || u > 1.0) {
        v = (-k1 + w) / (2.0 * k2);
        u = (h.x - f.x * v) / (e.x + g.x * v);
    }
    if (v < 0.0 || v > 1.0 || u < 0.0 || u > 1.0) {
        v = -1.0;
        u = -1.0;
    }

    return Vector2d(u, v);
}

extern "C"
JNIEXPORT void JNICALL Java_com_frank_family_CubeLayout_transformsCube(JNIEnv *env, jobject obj,
                                                                         const jint count,
                                                                         const jintArray index,
                                                                         const jobjectArray facePointsX,
                                                                         const jobjectArray facePointsY) {

    int original_point, pixel_point, X, Y;
    jint* picIndex = env->GetIntArrayElements((jintArray)index, 0);

    for (int i = 0; i < count; i++) {
        jobject myarrayx = env->GetObjectArrayElement(facePointsX, i);
        jobject myarrayy = env->GetObjectArrayElement(facePointsY, i);

        jfloat *datax = env->GetFloatArrayElements((jfloatArray) myarrayx, 0);
        jfloat *datay = env->GetFloatArrayElements((jfloatArray) myarrayy, 0);
        for (int j = 0; j < 4; j++) {
            FacePoints[i][j].x = datax[j];
            FacePoints[i][j].y = datay[j];
        }
    }

    bool isInside;
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            pixel_point = (width * y + x) * 4;
            Vector2d p(x, y);
            isInside = false;
            for (int i = 0; i < count; i++) {
                if (inquat(i, x, y)) {
                    isInside = true;
                    Vector2d v2 = invBilinear(p, i);
                    X = (int) ((float) width00 * v2.x);
                    Y = (int) ((float) height00 * v2.y);
                    original_point = (Y * width00 + X) * 4;
                    *(CubeBufOut + pixel_point) = *(CubeBuf[picIndex[i]] + original_point);
                    *(CubeBufOut + pixel_point + 1) = *(CubeBuf[picIndex[i]] + original_point + 1);
                    *(CubeBufOut + pixel_point + 2) = *(CubeBuf[picIndex[i]] + original_point + 2);
                    *(CubeBufOut + pixel_point + 3) = -1;
                }
            }
            if (!isInside) {
                int x_index = round(x / sx);
                int y_index = round(y / sy);

                original_point = (y_index * BGWidth + x_index) * 4;
                *(CubeBufOut + pixel_point) = *(BGBufOut + original_point);
                *(CubeBufOut + pixel_point + 1) = *(BGBufOut + original_point + 1);
                *(CubeBufOut + pixel_point + 2) = *(BGBufOut + original_point + 2);
                *(CubeBufOut + pixel_point + 3) = -1;
            }
        }
    }
}

extern "C"
JNIEXPORT void JNICALL Java_com_frank_family_MainActivity_EndDraw(JNIEnv *env, jobject obj) {

    env->ReleaseByteArrayElements(pAllBGData, BGBufOut, 0);

    env->ReleaseByteArrayElements(pCubeSrcData1, CubeBuf[0], 0);
    env->ReleaseByteArrayElements(pCubeSrcData2, CubeBuf[1], 0);
    env->ReleaseByteArrayElements(pCubeSrcData3, CubeBuf[2], 0);
    env->ReleaseByteArrayElements(pCubeSrcData4, CubeBuf[3], 0);
    env->ReleaseByteArrayElements(pCubeSrcData5, CubeBuf[4], 0);
    env->ReleaseByteArrayElements(pCubeSrcData6, CubeBuf[5], 0);
    env->ReleaseByteArrayElements(pCubeOutData, CubeBufOut, 0);

    env->ReleaseByteArrayElements(pBallSrcData, BallBuf, 0);
    env->ReleaseByteArrayElements(pFatherSrcData, FatherBuf, 0);
    env->ReleaseByteArrayElements(pMotherSrcData, MotherBuf, 0);
    env->ReleaseByteArrayElements(pBallOutData, BallBufOut, 0);
}
