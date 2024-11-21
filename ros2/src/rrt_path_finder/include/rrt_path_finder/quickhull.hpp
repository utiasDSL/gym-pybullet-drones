// Public domain code from Antti Kuukka
// https://github.com/akuukka/quickhull
// Many thanks to him.

#ifndef QUICKHULL_HPP
#define QUICKHULL_HPP

#include <iostream>
#include <cmath>
#include <limits>
#include <memory>
#include <algorithm>
#include <array>
#include <vector>
#include <deque>
#include <unordered_map>
#include <fstream>
#include <cassert>

namespace quickhull
{

    // ----------------------- BasicData Part -----------------------

    template <typename T>
    class Vector3
    {
    public:
        Vector3() = default;

        Vector3(const T &x, const T &y, const T &z) : x(x), y(y), z(z) {}

        T x, y, z;

        inline T dotProduct(const Vector3 &other) const
        {
            return x * other.x + y * other.y + z * other.z;
        }

        inline Vector3 operator-(const Vector3 &other) const
        {
            return Vector3(x - other.x, y - other.y, z - other.z);
        }

        inline Vector3 operator+(const Vector3 &other) const
        {
            return Vector3(x + other.x, y + other.y, z + other.z);
        }

        inline Vector3 &operator+=(const Vector3 &other)
        {
            x += other.x;
            y += other.y;
            z += other.z;
            return *this;
        }

        inline Vector3 &operator-=(const Vector3 &other)
        {
            x -= other.x;
            y -= other.y;
            z -= other.z;
            return *this;
        }

        inline Vector3 &operator*=(const T &c)
        {
            x *= c;
            y *= c;
            z *= c;
            return *this;
        }

        inline Vector3 &operator/=(const T &c)
        {
            x /= c;
            y /= c;
            z /= c;
            return *this;
        }

        inline Vector3 operator-() const
        {
            return Vector3(-x, -y, -z);
        }

        template <typename S>
        inline Vector3 operator*(const S &c) const
        {
            return Vector3(x * c, y * c, z * c);
        }

        template <typename S>
        inline Vector3 operator/(const S &c) const
        {
            return Vector3(x / c, y / c, z / c);
        }

        inline T getLengthSquared() const
        {
            return x * x + y * y + z * z;
        }

        inline bool operator!=(const Vector3 &o) const
        {
            return x != o.x || y != o.y || z != o.z;
        }

        inline T getSquaredDistanceTo(const Vector3 &other) const
        {
            const T dx = x - other.x;
            const T dy = y - other.y;
            const T dz = z - other.z;
            return dx * dx + dy * dy + dz * dz;
        }
    };

    // Overload also << operator for easy printing of debug data
    template <typename T>
    inline std::ostream &operator<<(std::ostream &os, const Vector3<T> &vec)
    {
        os << "(" << vec.x << "," << vec.y << "," << vec.z << ")";
        return os;
    }

    template <typename T>
    inline Vector3<T> operator*(T c, const Vector3<T> &v)
    {
        return Vector3<T>(v.x * c, v.y * c, v.z * c);
    }

    template <typename T>
    class Plane
    {
    public:
        Vector3<T> m_N;

        // Signed distance (if normal is of length 1) to the plane from origin
        T m_D;

        // Normal length squared
        T m_sqrNLength;

        inline bool isPointOnPositiveSide(const Vector3<T> &Q) const
        {
            T d = m_N.dotProduct(Q) + m_D;
            if (d >= 0)
                return true;
            return false;
        }

        Plane() = default;

        // Construct a plane using normal N and any point P on the plane
        Plane(const Vector3<T> &N, const Vector3<T> &P)
            : m_N(N), m_D(-N.dotProduct(P)),
              m_sqrNLength(m_N.x * m_N.x + m_N.y * m_N.y + m_N.z * m_N.z) {}
    };

    template <typename T>
    struct Ray
    {
        const Vector3<T> m_S;
        const Vector3<T> m_V;
        const T m_VInvLengthSquared;

        Ray(const Vector3<T> &S, const Vector3<T> &V)
            : m_S(S), m_V(V), m_VInvLengthSquared(T{1} / m_V.getLengthSquared()) {}
    };

    template <typename S>
    class Pool
    {
        std::vector<std::unique_ptr<S>> m_data;

    public:
        inline void clear()
        {
            m_data.clear();
        }

        inline void reclaim(std::unique_ptr<S> &ptr)
        {
            m_data.push_back(std::move(ptr));
        }

        inline std::unique_ptr<S> get()
        {
            if (m_data.empty())
            {
                return std::unique_ptr<S>(new S());
            }
            auto it = m_data.end() - 1;
            std::unique_ptr<S> r = std::move(*it);
            m_data.erase(it);
            return r;
        }
    };

    template <typename T>
    class VertexDataSource
    {
        const Vector3<T> *m_ptr;
        size_t m_count;

    public:
        VertexDataSource(const Vector3<T> *ptr, size_t count)
            : m_ptr(ptr), m_count(count) {}

        VertexDataSource(const std::vector<Vector3<T>> &vec)
            : m_ptr(&vec[0]), m_count(vec.size()) {}

        VertexDataSource()
            : m_ptr(nullptr), m_count(0) {}

        inline VertexDataSource &operator=(const VertexDataSource &other) = default;

        inline size_t size() const
        {
            return m_count;
        }

        inline const Vector3<T> &operator[](size_t index) const
        {
            return m_ptr[index];
        }

        inline const Vector3<T> *begin() const
        {
            return m_ptr;
        }

        inline const Vector3<T> *end() const
        {
            return m_ptr + m_count;
        }
    };

    // ----------------------- MeshBuilder Part -----------------------

    template <typename T>
    class MeshBuilder
    {
    public:
        struct HalfEdge
        {
            size_t m_endVertex;
            size_t m_opp;
            size_t m_face;
            size_t m_next;

            inline void disable()
            {
                m_endVertex = std::numeric_limits<size_t>::max();
            }

            inline bool isDisabled() const
            {
                return m_endVertex == std::numeric_limits<size_t>::max();
            }
        };

        struct Face
        {
            size_t m_he;
            Plane<T> m_P;
            T m_mostDistantPointDist;
            size_t m_mostDistantPoint;
            size_t m_visibilityCheckedOnIteration;
            std::uint8_t m_isVisibleFaceOnCurrentIteration : 1;
            std::uint8_t m_inFaceStack : 1;

            // Bit for each half edge assigned to this face,
            // each being 0 or 1 depending on whether the
            // edge belongs to horizon edge
            std::uint8_t m_horizonEdgesOnCurrentIteration : 3;
            std::unique_ptr<std::vector<size_t>> m_pointsOnPositiveSide;

            Face() : m_he(std::numeric_limits<size_t>::max()),
                     m_P(),
                     m_mostDistantPointDist(0),
                     m_mostDistantPoint(0),
                     m_visibilityCheckedOnIteration(0),
                     m_isVisibleFaceOnCurrentIteration(0),
                     m_inFaceStack(0),
                     m_horizonEdgesOnCurrentIteration(0) {}

            inline void disable()
            {
                m_he = std::numeric_limits<size_t>::max();
            }

            inline bool isDisabled() const
            {
                return m_he == std::numeric_limits<size_t>::max();
            }
        };

        // Mesh data
        std::vector<Face> m_faces;
        std::vector<HalfEdge> m_halfEdges;

        // When the mesh is modified and faces and half edges are
        // removed from it, we do not actually remove them from
        // the container vectors. Insted, they are marked as disabled
        // which means that the indices can be reused when we need to
        // add new faces and half edges to the mesh. We store the
        // free indices in the following vectors.
        std::vector<size_t> m_disabledFaces, m_disabledHalfEdges;

        inline size_t addFace()
        {
            if (m_disabledFaces.size())
            {
                size_t index = m_disabledFaces.back();
                auto &f = m_faces[index];
                assert(f.isDisabled());
                assert(!f.m_pointsOnPositiveSide);
                f.m_mostDistantPointDist = 0;
                m_disabledFaces.pop_back();
                return index;
            }
            m_faces.emplace_back();
            return m_faces.size() - 1;
        }

        inline size_t addHalfEdge()
        {
            if (m_disabledHalfEdges.size())
            {
                const size_t index = m_disabledHalfEdges.back();
                m_disabledHalfEdges.pop_back();
                return index;
            }
            m_halfEdges.emplace_back();
            return m_halfEdges.size() - 1;
        }

        // Mark a face as disabled and return a pointer
        // to the points that were on the positive of it.
        inline std::unique_ptr<std::vector<size_t>> disableFace(size_t faceIndex)
        {
            auto &f = m_faces[faceIndex];
            f.disable();
            m_disabledFaces.push_back(faceIndex);
            return std::move(f.m_pointsOnPositiveSide);
        }

        inline void disableHalfEdge(size_t heIndex)
        {
            auto &he = m_halfEdges[heIndex];
            he.disable();
            m_disabledHalfEdges.push_back(heIndex);
        }

        MeshBuilder() = default;

        // Create a mesh with initial tetrahedron ABCD.
        // Dot product of AB with the normal of triangle
        // ABC should be negative.
        inline void setup(size_t a, size_t b, size_t c, size_t d)
        {
            m_faces.clear();
            m_halfEdges.clear();
            m_disabledFaces.clear();
            m_disabledHalfEdges.clear();

            m_faces.reserve(4);
            m_halfEdges.reserve(12);

            // Create halfedges
            HalfEdge AB;
            AB.m_endVertex = b;
            AB.m_opp = 6;
            AB.m_face = 0;
            AB.m_next = 1;
            m_halfEdges.push_back(AB);

            HalfEdge BC;
            BC.m_endVertex = c;
            BC.m_opp = 9;
            BC.m_face = 0;
            BC.m_next = 2;
            m_halfEdges.push_back(BC);

            HalfEdge CA;
            CA.m_endVertex = a;
            CA.m_opp = 3;
            CA.m_face = 0;
            CA.m_next = 0;
            m_halfEdges.push_back(CA);

            HalfEdge AC;
            AC.m_endVertex = c;
            AC.m_opp = 2;
            AC.m_face = 1;
            AC.m_next = 4;
            m_halfEdges.push_back(AC);

            HalfEdge CD;
            CD.m_endVertex = d;
            CD.m_opp = 11;
            CD.m_face = 1;
            CD.m_next = 5;
            m_halfEdges.push_back(CD);

            HalfEdge DA;
            DA.m_endVertex = a;
            DA.m_opp = 7;
            DA.m_face = 1;
            DA.m_next = 3;
            m_halfEdges.push_back(DA);

            HalfEdge BA;
            BA.m_endVertex = a;
            BA.m_opp = 0;
            BA.m_face = 2;
            BA.m_next = 7;
            m_halfEdges.push_back(BA);

            HalfEdge AD;
            AD.m_endVertex = d;
            AD.m_opp = 5;
            AD.m_face = 2;
            AD.m_next = 8;
            m_halfEdges.push_back(AD);

            HalfEdge DB;
            DB.m_endVertex = b;
            DB.m_opp = 10;
            DB.m_face = 2;
            DB.m_next = 6;
            m_halfEdges.push_back(DB);

            HalfEdge CB;
            CB.m_endVertex = b;
            CB.m_opp = 1;
            CB.m_face = 3;
            CB.m_next = 10;
            m_halfEdges.push_back(CB);

            HalfEdge BD;
            BD.m_endVertex = d;
            BD.m_opp = 8;
            BD.m_face = 3;
            BD.m_next = 11;
            m_halfEdges.push_back(BD);

            HalfEdge DC;
            DC.m_endVertex = c;
            DC.m_opp = 4;
            DC.m_face = 3;
            DC.m_next = 9;
            m_halfEdges.push_back(DC);

            // Create faces
            Face ABC;
            ABC.m_he = 0;
            m_faces.push_back(std::move(ABC));

            Face ACD;
            ACD.m_he = 3;
            m_faces.push_back(std::move(ACD));

            Face BAD;
            BAD.m_he = 6;
            m_faces.push_back(std::move(BAD));

            Face CBD;
            CBD.m_he = 9;
            m_faces.push_back(std::move(CBD));
        }

        inline std::array<size_t, 3> getVertexIndicesOfFace(const Face &f) const
        {
            std::array<size_t, 3> v;
            const HalfEdge *he = &m_halfEdges[f.m_he];
            v[0] = he->m_endVertex;
            he = &m_halfEdges[he->m_next];
            v[1] = he->m_endVertex;
            he = &m_halfEdges[he->m_next];
            v[2] = he->m_endVertex;
            return v;
        }

        inline std::array<size_t, 2> getVertexIndicesOfHalfEdge(const HalfEdge &he) const
        {
            return {m_halfEdges[he.m_opp].m_endVertex, he.m_endVertex};
        }

        inline std::array<size_t, 3> getHalfEdgeIndicesOfFace(const Face &f) const
        {
            return {f.m_he, m_halfEdges[f.m_he].m_next, m_halfEdges[m_halfEdges[f.m_he].m_next].m_next};
        }
    };

    // ----------------------- MathUtils Part -----------------------

    namespace mathutils
    {

        template <typename T>
        inline T getSquaredDistanceBetweenPointAndRay(const Vector3<T> &p, const Ray<T> &r)
        {
            const Vector3<T> s = p - r.m_S;
            T t = s.dotProduct(r.m_V);
            return s.getLengthSquared() - t * t * r.m_VInvLengthSquared;
        }

        // Note that the unit of distance returned is relative
        // to plane's normal's length (divide by N.getNormalized()
        // if needed to get the "real" distance).
        template <typename T>
        inline T getSignedDistanceToPlane(const Vector3<T> &v, const Plane<T> &p)
        {
            return p.m_N.dotProduct(v) + p.m_D;
        }

        template <typename T>
        inline Vector3<T> getTriangleNormal(const Vector3<T> &a,
                                            const Vector3<T> &b,
                                            const Vector3<T> &c)
        {
            // We want to get (a-c).crossProduct(b-c)
            // without constructing temp vectors
            T x = a.x - c.x;
            T y = a.y - c.y;
            T z = a.z - c.z;
            T rhsx = b.x - c.x;
            T rhsy = b.y - c.y;
            T rhsz = b.z - c.z;
            T px = y * rhsz - z * rhsy;
            T py = z * rhsx - x * rhsz;
            T pz = x * rhsy - y * rhsx;
            return Vector3<T>(px, py, pz);
        }

    } // namespace mathutils

    // ----------------------- HalfEdgeMesh Part -----------------------

    template <typename FloatType, typename IndexType>
    class HalfEdgeMesh
    {
    public:
        struct HalfEdge
        {
            IndexType m_endVertex;
            IndexType m_opp;
            IndexType m_face;
            IndexType m_next;
        };

        struct Face
        {
            // Index of one of the half edges of this face
            IndexType m_halfEdgeIndex;
        };

        std::vector<Vector3<FloatType>> m_vertices;
        std::vector<Face> m_faces;
        std::vector<HalfEdge> m_halfEdges;

        HalfEdgeMesh(const MeshBuilder<FloatType> &builderObject,
                     const VertexDataSource<FloatType> &vertexData)
        {
            std::unordered_map<IndexType, IndexType> faceMapping;
            std::unordered_map<IndexType, IndexType> halfEdgeMapping;
            std::unordered_map<IndexType, IndexType> vertexMapping;

            size_t i = 0;
            for (const auto &face : builderObject.m_faces)
            {
                if (!face.isDisabled())
                {
                    m_faces.push_back({static_cast<IndexType>(face.m_he)});
                    faceMapping[i] = m_faces.size() - 1;

                    const auto heIndices = builderObject.getHalfEdgeIndicesOfFace(face);
                    for (const auto heIndex : heIndices)
                    {
                        const IndexType vertexIndex = builderObject.m_halfEdges[heIndex].m_endVertex;
                        if (vertexMapping.count(vertexIndex) == 0)
                        {
                            m_vertices.push_back(vertexData[vertexIndex]);
                            vertexMapping[vertexIndex] = m_vertices.size() - 1;
                        }
                    }
                }
                i++;
            }

            i = 0;
            for (const auto &halfEdge : builderObject.m_halfEdges)
            {
                if (!halfEdge.isDisabled())
                {
                    m_halfEdges.push_back({static_cast<IndexType>(halfEdge.m_endVertex),
                                           static_cast<IndexType>(halfEdge.m_opp),
                                           static_cast<IndexType>(halfEdge.m_face),
                                           static_cast<IndexType>(halfEdge.m_next)});
                    halfEdgeMapping[i] = m_halfEdges.size() - 1;
                }
                i++;
            }

            for (auto &face : m_faces)
            {
                assert(halfEdgeMapping.count(face.m_halfEdgeIndex) == 1);
                face.m_halfEdgeIndex = halfEdgeMapping[face.m_halfEdgeIndex];
            }

            for (auto &he : m_halfEdges)
            {
                he.m_face = faceMapping[he.m_face];
                he.m_opp = halfEdgeMapping[he.m_opp];
                he.m_next = halfEdgeMapping[he.m_next];
                he.m_endVertex = vertexMapping[he.m_endVertex];
            }
        }
    };

    // ----------------------- Convex Hull Part -----------------------

    template <typename T>
    class ConvexHull
    {
        std::unique_ptr<std::vector<Vector3<T>>> m_optimizedVertexBuffer;
        VertexDataSource<T> m_vertices;
        std::vector<size_t> m_indices;

    public:
        ConvexHull() {}

        // Copy constructor
        ConvexHull(const ConvexHull &o)
        {
            m_indices = o.m_indices;
            if (o.m_optimizedVertexBuffer)
            {
                m_optimizedVertexBuffer.reset(new std::vector<Vector3<T>>(*o.m_optimizedVertexBuffer));
                m_vertices = VertexDataSource<T>(*m_optimizedVertexBuffer);
            }
            else
            {
                m_vertices = o.m_vertices;
            }
        }

        inline ConvexHull &operator=(const ConvexHull &o)
        {
            if (&o == this)
            {
                return *this;
            }
            m_indices = o.m_indices;
            if (o.m_optimizedVertexBuffer)
            {
                m_optimizedVertexBuffer.reset(new std::vector<Vector3<T>>(*o.m_optimizedVertexBuffer));
                m_vertices = VertexDataSource<T>(*m_optimizedVertexBuffer);
            }
            else
            {
                m_vertices = o.m_vertices;
            }
            return *this;
        }

        ConvexHull(ConvexHull &&o)
        {
            m_indices = std::move(o.m_indices);
            if (o.m_optimizedVertexBuffer)
            {
                m_optimizedVertexBuffer = std::move(o.m_optimizedVertexBuffer);
                o.m_vertices = VertexDataSource<T>();
                m_vertices = VertexDataSource<T>(*m_optimizedVertexBuffer);
            }
            else
            {
                m_vertices = o.m_vertices;
            }
        }

        inline ConvexHull &operator=(ConvexHull &&o)
        {
            if (&o == this)
            {
                return *this;
            }
            m_indices = std::move(o.m_indices);
            if (o.m_optimizedVertexBuffer)
            {
                m_optimizedVertexBuffer = std::move(o.m_optimizedVertexBuffer);
                o.m_vertices = VertexDataSource<T>();
                m_vertices = VertexDataSource<T>(*m_optimizedVertexBuffer);
            }
            else
            {
                m_vertices = o.m_vertices;
            }
            return *this;
        }

        // Construct vertex and index buffers from
        // half edge mesh and pointcloud
        ConvexHull(const MeshBuilder<T> &mesh,
                   const VertexDataSource<T> &pointCloud,
                   bool CCW,
                   bool useOriginalIndices)
        {
            if (!useOriginalIndices)
            {
                m_optimizedVertexBuffer.reset(new std::vector<Vector3<T>>());
            }

            std::vector<std::uint8_t> faceProcessed(mesh.m_faces.size(), 0);
            std::vector<size_t> faceStack;
            // Map vertex indices from original point cloud
            // to the new mesh vertex indices
            std::unordered_map<size_t, size_t> vertexIndexMapping;
            for (size_t i = 0; i < mesh.m_faces.size(); i++)
            {
                if (!mesh.m_faces[i].isDisabled())
                {
                    faceStack.push_back(i);
                    break;
                }
            }
            if (faceStack.size() == 0)
            {
                return;
            }

            const size_t iCCW = CCW ? 1 : 0;
            const size_t finalMeshFaceCount = mesh.m_faces.size() - mesh.m_disabledFaces.size();
            m_indices.reserve(finalMeshFaceCount * 3);

            while (faceStack.size())
            {
                auto it = faceStack.end() - 1;
                size_t top = *it;
                assert(!mesh.m_faces[top].isDisabled());
                faceStack.erase(it);
                if (faceProcessed[top])
                {
                    continue;
                }
                else
                {
                    faceProcessed[top] = 1;
                    auto halfEdges = mesh.getHalfEdgeIndicesOfFace(mesh.m_faces[top]);
                    size_t adjacent[] = {mesh.m_halfEdges[mesh.m_halfEdges[halfEdges[0]].m_opp].m_face,
                                         mesh.m_halfEdges[mesh.m_halfEdges[halfEdges[1]].m_opp].m_face,
                                         mesh.m_halfEdges[mesh.m_halfEdges[halfEdges[2]].m_opp].m_face};
                    for (auto a : adjacent)
                    {
                        if (!faceProcessed[a] && !mesh.m_faces[a].isDisabled())
                        {
                            faceStack.push_back(a);
                        }
                    }
                    auto vertices = mesh.getVertexIndicesOfFace(mesh.m_faces[top]);
                    if (!useOriginalIndices)
                    {
                        for (auto &v : vertices)
                        {
                            auto itV = vertexIndexMapping.find(v);
                            if (itV == vertexIndexMapping.end())
                            {
                                m_optimizedVertexBuffer->push_back(pointCloud[v]);
                                vertexIndexMapping[v] = m_optimizedVertexBuffer->size() - 1;
                                v = m_optimizedVertexBuffer->size() - 1;
                            }
                            else
                            {
                                v = itV->second;
                            }
                        }
                    }
                    m_indices.push_back(vertices[0]);
                    m_indices.push_back(vertices[1 + iCCW]);
                    m_indices.push_back(vertices[2 - iCCW]);
                }
            }

            if (!useOriginalIndices)
            {
                m_vertices = VertexDataSource<T>(*m_optimizedVertexBuffer);
            }
            else
            {
                m_vertices = pointCloud;
            }
        }

        inline std::vector<size_t> &getIndexBuffer()
        {
            return m_indices;
        }

        inline const std::vector<size_t> &getIndexBuffer() const
        {
            return m_indices;
        }

        inline VertexDataSource<T> &getVertexBuffer()
        {
            return m_vertices;
        }

        inline const VertexDataSource<T> &getVertexBuffer() const
        {
            return m_vertices;
        }

        // Export the mesh to a Waveform OBJ file
        inline void writeWaveformOBJ(const std::string &filename,
                                     const std::string &objectName = "quickhull") const
        {
            std::ofstream objFile;
            objFile.open(filename);
            objFile << "o " << objectName << "\n";
            for (const auto &v : getVertexBuffer())
            {
                objFile << "v " << v.x << " " << v.y << " " << v.z << "\n";
            }
            const auto &indBuf = getIndexBuffer();
            size_t triangleCount = indBuf.size() / 3;
            for (size_t i = 0; i < triangleCount; i++)
            {
                objFile << "f " << indBuf[i * 3] + 1 << " "
                        << indBuf[i * 3 + 1] + 1 << " "
                        << indBuf[i * 3 + 2] + 1 << "\n";
            }
            objFile.close();
        }
    };

    // ----------------------- Quick Hull Part -----------------------

    // Implementation of the 3D QuickHull algorithm by Antti Kuukka
    //
    // No copyrights. What follows is 100% Public Domain.
    //
    //
    //
    // INPUT:  a list of points in 3D space (for example, vertices of a 3D mesh)
    //
    // OUTPUT: a ConvexHull object which provides vertex and index buffers of
    // the generated convex hull as a triangle mesh.
    //
    //
    //
    // The implementation is thread-safe if each thread is using its own
    // QuickHull object.
    //
    //
    // SUMMARY OF THE ALGORITHM:
    //         - Create initial simplex (tetrahedron) using extreme points.
    //           We have four faces now and they form a convex mesh M.
    //         - For each point, assign them to the first face for which they
    //           are on the positive side of (so each point is assigned to at
    //           most one face). Points inside the initial tetrahedron are left
    //           behind now and no longer affect the calculations.
    //         - Add all faces that have points assigned to them to Face Stack.
    //         - Iterate until Face Stack is empty:
    //              - Pop topmost face F from the stack
    //              - From the points assigned to F, pick the point P that is
    //                farthest away from the plane defined by F.
    //              - Find all faces of M that have P on their positive side.
    //                Let us call these the "visible faces".
    //              - Because of the way M is constructed, these faces are connected.
    //                Solve their horizon edge loop.
    //				- "Extrude to P": Create new faces by connecting P with the points
    //                 belonging to the horizon edge. Add the new faces to M and remove
    //                 the visible faces from M.
    //              - Each point that was assigned to visible faces is now assigned
    //                to at most one of the newly created faces.
    //              - Those new faces that have points assigned to them are added
    //                to the top of Face Stack.
    //          - M is now the convex hull.
    //
    // TO DO:
    //  - Implement a proper 2D QuickHull and use that to solve the degenerate 2D case
    //    (when all the points lie on the same plane in 3D space).

    struct DiagnosticsData
    {
        // How many times QuickHull failed to solve the horizon edge. Failures lead to
        // degenerated convex hulls.
        size_t m_failedHorizonEdges;

        DiagnosticsData() : m_failedHorizonEdges(0) {}
    };

    template <typename T>
    inline T defaultEps()
    {
        return T{1} / T{10000000};
    }

    template <typename T>
    class QuickHull
    {
        using vec3 = Vector3<T>;

        T m_epsilon, m_epsilonSquared, m_scale;
        bool m_planar;
        std::vector<vec3> m_planarPointCloudTemp;
        VertexDataSource<T> m_vertexData;
        MeshBuilder<T> m_mesh;
        std::array<size_t, 6> m_extremeValues;
        DiagnosticsData m_diagnostics;

        // Temporary variables used during iteration process
        std::vector<size_t> m_newFaceIndices;
        std::vector<size_t> m_newHalfEdgeIndices;
        std::vector<std::unique_ptr<std::vector<size_t>>> m_disabledFacePointVectors;
        std::vector<size_t> m_visibleFaces;
        std::vector<size_t> m_horizonEdges;
        struct FaceData
        {
            size_t m_faceIndex;
            // If the face turns out not to be visible, this half edge will be marked as horizon edge
            size_t m_enteredFromHalfEdge;
            FaceData(size_t fi, size_t he) : m_faceIndex(fi), m_enteredFromHalfEdge(he) {}
        };
        std::vector<FaceData> m_possiblyVisibleFaces;
        std::deque<size_t> m_faceList;

        // Create a half edge mesh representing the base tetrahedron from which the QuickHull
        // iteration proceeds. m_extremeValues must be properly set up when this is called.
        inline void setupInitialTetrahedron()
        {
            const size_t vertexCount = m_vertexData.size();

            // If we have at most 4 points, just return a degenerate tetrahedron:
            if (vertexCount <= 4)
            {
                size_t v[4] = {0,
                               std::min((size_t)1, vertexCount - 1),
                               std::min((size_t)2, vertexCount - 1),
                               std::min((size_t)3, vertexCount - 1)};
                const Vector3<T> N = mathutils::getTriangleNormal(m_vertexData[v[0]],
                                                                  m_vertexData[v[1]],
                                                                  m_vertexData[v[2]]);
                const Plane<T> trianglePlane(N, m_vertexData[v[0]]);
                if (trianglePlane.isPointOnPositiveSide(m_vertexData[v[3]]))
                {
                    std::swap(v[0], v[1]);
                }
                return m_mesh.setup(v[0], v[1], v[2], v[3]);
            }

            // Find two most distant extreme points.
            T maxD = m_epsilonSquared;
            std::pair<size_t, size_t> selectedPoints;
            for (size_t i = 0; i < 6; i++)
            {
                for (size_t j = i + 1; j < 6; j++)
                {
                    const T d = m_vertexData[m_extremeValues[i]].getSquaredDistanceTo(m_vertexData[m_extremeValues[j]]);
                    if (d > maxD)
                    {
                        maxD = d;
                        selectedPoints = {m_extremeValues[i], m_extremeValues[j]};
                    }
                }
            }
            if (maxD == m_epsilonSquared)
            {
                // A degenerate case: the point cloud seems to consists of a single point
                return m_mesh.setup(0,
                                    std::min((size_t)1, vertexCount - 1),
                                    std::min((size_t)2, vertexCount - 1),
                                    std::min((size_t)3, vertexCount - 1));
            }
            assert(selectedPoints.first != selectedPoints.second);

            // Find the most distant point to the line between the two chosen extreme points.
            const Ray<T> r(m_vertexData[selectedPoints.first],
                           (m_vertexData[selectedPoints.second] - m_vertexData[selectedPoints.first]));
            maxD = m_epsilonSquared;
            size_t maxI = std::numeric_limits<size_t>::max();
            const size_t vCount = m_vertexData.size();
            for (size_t i = 0; i < vCount; i++)
            {
                const T distToRay = mathutils::getSquaredDistanceBetweenPointAndRay(m_vertexData[i], r);
                if (distToRay > maxD)
                {
                    maxD = distToRay;
                    maxI = i;
                }
            }
            if (maxD == m_epsilonSquared)
            {
                // It appears that the point cloud belongs to a 1 dimensional
                // subspace of R^3: convex hull has no volume => return a thin
                // triangle. Pick any point other than selectedPoints.first and
                // selectedPoints.second as the third point of the triangle

                auto it = m_vertexData.begin();
                for (; it != m_vertexData.end() &&
                       !(*it != m_vertexData[selectedPoints.first] && *it != m_vertexData[selectedPoints.second]);
                     it++)
                {
                }
                const size_t thirdPoint = (it == m_vertexData.end()) ? selectedPoints.first : std::distance(m_vertexData.begin(), it);

                for (it = m_vertexData.begin();
                     it != m_vertexData.end() &&
                     !(*it != m_vertexData[selectedPoints.first] &&
                       *it != m_vertexData[selectedPoints.second] &&
                       *it != m_vertexData[thirdPoint]);
                     it++)
                {
                }
                const size_t fourthPoint = (it == m_vertexData.end()) ? selectedPoints.first : std::distance(m_vertexData.begin(), it);

                return m_mesh.setup(selectedPoints.first, selectedPoints.second, thirdPoint, fourthPoint);
            }

            // These three points form the base triangle for our tetrahedron.
            assert(selectedPoints.first != maxI && selectedPoints.second != maxI);
            std::array<size_t, 3> baseTriangle{selectedPoints.first, selectedPoints.second, maxI};
            const Vector3<T> baseTriangleVertices[] = {m_vertexData[baseTriangle[0]],
                                                       m_vertexData[baseTriangle[1]],
                                                       m_vertexData[baseTriangle[2]]};

            // Next step is to find the 4th vertex of the tetrahedron. We naturally
            // choose the point farthest away from the triangle plane.
            maxD = m_epsilon;
            maxI = 0;
            const Vector3<T> N = mathutils::getTriangleNormal(baseTriangleVertices[0],
                                                              baseTriangleVertices[1],
                                                              baseTriangleVertices[2]);
            Plane<T> trianglePlane(N, baseTriangleVertices[0]);
            for (size_t i = 0; i < vCount; i++)
            {
                const T d = abs(mathutils::getSignedDistanceToPlane(m_vertexData[i], trianglePlane));
                if (d > maxD)
                {
                    maxD = d;
                    maxI = i;
                }
            }
            if (maxD == m_epsilon)
            {
                // All the points seem to lie on a 2D subspace of R^3. How to handle this? Well, let's add
                // one extra point to the point cloud so that the convex hull will have volume.
                m_planar = true;
                const vec3 N1 = mathutils::getTriangleNormal(baseTriangleVertices[1],
                                                             baseTriangleVertices[2],
                                                             baseTriangleVertices[0]);
                m_planarPointCloudTemp.clear();
                m_planarPointCloudTemp.insert(m_planarPointCloudTemp.begin(),
                                              m_vertexData.begin(),
                                              m_vertexData.end());
                const vec3 extraPoint = N1 + m_vertexData[0];
                m_planarPointCloudTemp.push_back(extraPoint);
                maxI = m_planarPointCloudTemp.size() - 1;
                m_vertexData = VertexDataSource<T>(m_planarPointCloudTemp);
            }

            // Enforce CCW orientation (if user prefers clockwise orientation,
            // swap two vertices in each triangle when final mesh is created)
            const Plane<T> triPlane(N, baseTriangleVertices[0]);
            if (triPlane.isPointOnPositiveSide(m_vertexData[maxI]))
            {
                std::swap(baseTriangle[0], baseTriangle[1]);
            }

            // Create a tetrahedron half edge mesh and compute planes defined by each triangle
            m_mesh.setup(baseTriangle[0], baseTriangle[1], baseTriangle[2], maxI);
            for (auto &f : m_mesh.m_faces)
            {
                auto v = m_mesh.getVertexIndicesOfFace(f);
                const Vector3<T> &va = m_vertexData[v[0]];
                const Vector3<T> &vb = m_vertexData[v[1]];
                const Vector3<T> &vc = m_vertexData[v[2]];
                const Vector3<T> N1 = mathutils::getTriangleNormal(va, vb, vc);
                const Plane<T> plane(N1, va);
                f.m_P = plane;
            }

            // Finally we assign a face for each vertex outside the tetrahedron
            // (vertices inside the tetrahedron have no role anymore)
            for (size_t i = 0; i < vCount; i++)
            {
                for (auto &face : m_mesh.m_faces)
                {
                    if (addPointToFace(face, i))
                    {
                        break;
                    }
                }
            }
        }

        // Given a list of half edges, try to rearrange them so
        // that they form a loop. Return true on success.
        inline bool reorderHorizonEdges(std::vector<size_t> &horizonEdges)
        {
            const size_t horizonEdgeCount = horizonEdges.size();
            for (size_t i = 0; i < horizonEdgeCount - 1; i++)
            {
                const size_t endVertex = m_mesh.m_halfEdges[horizonEdges[i]].m_endVertex;
                bool foundNext = false;
                for (size_t j = i + 1; j < horizonEdgeCount; j++)
                {
                    const size_t beginVertex = m_mesh.m_halfEdges[m_mesh.m_halfEdges[horizonEdges[j]].m_opp].m_endVertex;
                    if (beginVertex == endVertex)
                    {
                        std::swap(horizonEdges[i + 1], horizonEdges[j]);
                        foundNext = true;
                        break;
                    }
                }
                if (!foundNext)
                {
                    return false;
                }
            }
            assert(m_mesh.m_halfEdges[horizonEdges[horizonEdges.size() - 1]].m_endVertex == m_mesh.m_halfEdges[m_mesh.m_halfEdges[horizonEdges[0]].m_opp].m_endVertex);
            return true;
        }

        // Find indices of extreme values (max x, min x, max y, min y, max z, min z) for the given point cloud
        inline std::array<size_t, 6> getExtremeValues()
        {
            std::array<size_t, 6> outIndices{0, 0, 0, 0, 0, 0};
            T extremeVals[6] = {m_vertexData[0].x,
                                m_vertexData[0].x,
                                m_vertexData[0].y,
                                m_vertexData[0].y,
                                m_vertexData[0].z,
                                m_vertexData[0].z};
            const size_t vCount = m_vertexData.size();
            for (size_t i = 1; i < vCount; i++)
            {
                const Vector3<T> &pos = m_vertexData[i];
                if (pos.x > extremeVals[0])
                {
                    extremeVals[0] = pos.x;
                    outIndices[0] = i;
                }
                else if (pos.x < extremeVals[1])
                {
                    extremeVals[1] = pos.x;
                    outIndices[1] = i;
                }
                if (pos.y > extremeVals[2])
                {
                    extremeVals[2] = pos.y;
                    outIndices[2] = i;
                }
                else if (pos.y < extremeVals[3])
                {
                    extremeVals[3] = pos.y;
                    outIndices[3] = i;
                }
                if (pos.z > extremeVals[4])
                {
                    extremeVals[4] = pos.z;
                    outIndices[4] = i;
                }
                else if (pos.z < extremeVals[5])
                {
                    extremeVals[5] = pos.z;
                    outIndices[5] = i;
                }
            }
            return outIndices;
        }

        // Compute scale of the vertex data.
        inline T getScale(const std::array<size_t, 6> &extremeValues)
        {
            T s = 0;
            s = std::max(s, std::abs(*((const T *)(&m_vertexData[extremeValues[0]]) + 0)));
            s = std::max(s, std::abs(*((const T *)(&m_vertexData[extremeValues[1]]) + 0)));
            s = std::max(s, std::abs(*((const T *)(&m_vertexData[extremeValues[2]]) + 1)));
            s = std::max(s, std::abs(*((const T *)(&m_vertexData[extremeValues[3]]) + 1)));
            s = std::max(s, std::abs(*((const T *)(&m_vertexData[extremeValues[4]]) + 2)));
            s = std::max(s, std::abs(*((const T *)(&m_vertexData[extremeValues[5]]) + 2)));

            return s;
        }

        // Each face contains a unique pointer to a vector of indices.
        // However, many - often most - faces do not have any points
        // on the positive side of them especially at the the end of
        // the iteration. When a face is removed from the mesh, its
        // associated point vector, if such exists, is moved to the
        // index vector pool, and when we need to add new faces with
        // points on the positive side to the mesh, we reuse these
        // vectors. This reduces the amount of std::vectors we have
        // to deal with, and impact on performance is remarkable.
        Pool<std::vector<size_t>> m_indexVectorPool;
        inline std::unique_ptr<std::vector<size_t>> getIndexVectorFromPool();
        inline void reclaimToIndexVectorPool(std::unique_ptr<std::vector<size_t>> &ptr);

        // Associates a point with a face if the point resides on the positive
        // side of the plane. Returns true if the points was on the positive side.
        inline bool addPointToFace(typename MeshBuilder<T>::Face &f, size_t pointIndex);

        // This will update m_mesh from which we create the ConvexHull
        // object that getConvexHull function returns
        inline void createConvexHalfEdgeMesh()
        {
            m_visibleFaces.clear();
            m_horizonEdges.clear();
            m_possiblyVisibleFaces.clear();

            // Compute base tetrahedron
            setupInitialTetrahedron();
            assert(m_mesh.m_faces.size() == 4);

            // Init face stack with those faces that have points assigned to them
            m_faceList.clear();
            auto &f = m_mesh.m_faces;

            if (f[0].m_pointsOnPositiveSide &&
                f[0].m_pointsOnPositiveSide->size() > 0)
            {
                m_faceList.push_back(0);
                f[0].m_inFaceStack = 1;
            }
            if (f[1].m_pointsOnPositiveSide &&
                f[1].m_pointsOnPositiveSide->size() > 0)
            {
                m_faceList.push_back(1);
                f[1].m_inFaceStack = 1;
            }
            if (f[2].m_pointsOnPositiveSide &&
                f[2].m_pointsOnPositiveSide->size() > 0)
            {
                m_faceList.push_back(2);
                f[2].m_inFaceStack = 1;
            }
            if (f[3].m_pointsOnPositiveSide &&
                f[3].m_pointsOnPositiveSide->size() > 0)
            {
                m_faceList.push_back(3);
                f[3].m_inFaceStack = 1;
            }

            // Process faces until the face list is empty.
            size_t iter = 0;
            while (!m_faceList.empty())
            {
                iter++;
                if (iter == std::numeric_limits<size_t>::max())
                {
                    // Visible face traversal marks visited faces with iteration counter
                    //(to mark that the face has been visited on this iteration) and the
                    // max value represents unvisited faces. At this point we have to reset
                    // iteration counter. This shouldn't be an issue on 64 bit machines.
                    iter = 0;
                }

                const size_t topFaceIndex = m_faceList.front();
                m_faceList.pop_front();

                auto &tf = m_mesh.m_faces[topFaceIndex];
                tf.m_inFaceStack = 0;

                assert(!tf.m_pointsOnPositiveSide || tf.m_pointsOnPositiveSide->size() > 0);
                if (!tf.m_pointsOnPositiveSide || tf.isDisabled())
                {
                    continue;
                }

                // Pick the most distant point to this triangle plane
                // as the point to which we extrude
                const vec3 &activePoint = m_vertexData[tf.m_mostDistantPoint];
                const size_t activePointIndex = tf.m_mostDistantPoint;

                // Find out the faces that have our active point on their positive side
                // (these are the "visible faces"). The face on top of the stack of course
                // is one of them. At the same time, we create a list of horizon edges.
                m_horizonEdges.clear();
                m_possiblyVisibleFaces.clear();
                m_visibleFaces.clear();
                m_possiblyVisibleFaces.emplace_back(topFaceIndex, std::numeric_limits<size_t>::max());
                while (m_possiblyVisibleFaces.size())
                {
                    const auto faceData = m_possiblyVisibleFaces.back();
                    m_possiblyVisibleFaces.pop_back();
                    auto &pvf = m_mesh.m_faces[faceData.m_faceIndex];
                    assert(!pvf.isDisabled());

                    if (pvf.m_visibilityCheckedOnIteration == iter)
                    {
                        if (pvf.m_isVisibleFaceOnCurrentIteration)
                        {
                            continue;
                        }
                    }
                    else
                    {
                        const Plane<T> &P = pvf.m_P;
                        pvf.m_visibilityCheckedOnIteration = iter;
                        const T d = P.m_N.dotProduct(activePoint) + P.m_D;
                        if (d > 0)
                        {
                            pvf.m_isVisibleFaceOnCurrentIteration = 1;
                            pvf.m_horizonEdgesOnCurrentIteration = 0;
                            m_visibleFaces.push_back(faceData.m_faceIndex);
                            for (auto heIndex : m_mesh.getHalfEdgeIndicesOfFace(pvf))
                            {
                                if (m_mesh.m_halfEdges[heIndex].m_opp != faceData.m_enteredFromHalfEdge)
                                {
                                    m_possiblyVisibleFaces.emplace_back(m_mesh.m_halfEdges[m_mesh.m_halfEdges[heIndex].m_opp].m_face, heIndex);
                                }
                            }
                            continue;
                        }
                        assert(faceData.m_faceIndex != topFaceIndex);
                    }

                    // The face is not visible. Therefore, the halfedge
                    // we came from is part of the horizon edge.
                    pvf.m_isVisibleFaceOnCurrentIteration = 0;
                    m_horizonEdges.push_back(faceData.m_enteredFromHalfEdge);
                    // Store which half edge is the horizon edge. The other half edges of
                    // the face will not be part of the final mesh so their data slots can by recycled.
                    const auto halfEdges = m_mesh.getHalfEdgeIndicesOfFace(m_mesh.m_faces[m_mesh.m_halfEdges[faceData.m_enteredFromHalfEdge].m_face]);
                    const std::int8_t ind = (halfEdges[0] == faceData.m_enteredFromHalfEdge) ? 0 : (halfEdges[1] == faceData.m_enteredFromHalfEdge ? 1 : 2);
                    m_mesh.m_faces[m_mesh.m_halfEdges[faceData.m_enteredFromHalfEdge].m_face].m_horizonEdgesOnCurrentIteration |= (1 << ind);
                }
                const size_t horizonEdgeCount = m_horizonEdges.size();

                // Order horizon edges so that they form a loop. This may fail due to numerical
                // instability in which case we give up trying to solve horizon edge for this
                // point and accept a minor degeneration in the convex hull.
                if (!reorderHorizonEdges(m_horizonEdges))
                {
                    m_diagnostics.m_failedHorizonEdges++;
                    std::cerr << "Failed to solve horizon edge." << std::endl;
                    auto it = std::find(tf.m_pointsOnPositiveSide->begin(), tf.m_pointsOnPositiveSide->end(), activePointIndex);
                    tf.m_pointsOnPositiveSide->erase(it);
                    if (tf.m_pointsOnPositiveSide->size() == 0)
                    {
                        reclaimToIndexVectorPool(tf.m_pointsOnPositiveSide);
                    }
                    continue;
                }

                // Except for the horizon edges, all half edges of the visible faces
                // can be marked as disabled. Their data slots will be reused.
                // The faces will be disabled as well, but we need to remember the
                // points that were on the positive side of them - therefore we
                // save pointers to them.
                m_newFaceIndices.clear();
                m_newHalfEdgeIndices.clear();
                m_disabledFacePointVectors.clear();
                size_t disableCounter = 0;
                for (auto faceIndex : m_visibleFaces)
                {
                    auto &disabledFace = m_mesh.m_faces[faceIndex];
                    auto halfEdges = m_mesh.getHalfEdgeIndicesOfFace(disabledFace);
                    for (size_t j = 0; j < 3; j++)
                    {
                        if ((disabledFace.m_horizonEdgesOnCurrentIteration & (1 << j)) == 0)
                        {
                            if (disableCounter < horizonEdgeCount * 2)
                            {
                                // Use on this iteration
                                m_newHalfEdgeIndices.push_back(halfEdges[j]);
                                disableCounter++;
                            }
                            else
                            {
                                // Mark for reusal on later iteration step
                                m_mesh.disableHalfEdge(halfEdges[j]);
                            }
                        }
                    }
                    // Disable the face, but retain pointer to the points that were
                    // on the positive side of it. We need to assign those points to
                    // the new faces we create shortly.
                    auto t = std::move(m_mesh.disableFace(faceIndex));
                    if (t)
                    {
                        // Because we should not assign point vectors to faces unless needed...
                        assert(t->size());
                        m_disabledFacePointVectors.push_back(std::move(t));
                    }
                }
                if (disableCounter < horizonEdgeCount * 2)
                {
                    const size_t newHalfEdgesNeeded = horizonEdgeCount * 2 - disableCounter;
                    for (size_t i = 0; i < newHalfEdgesNeeded; i++)
                    {
                        m_newHalfEdgeIndices.push_back(m_mesh.addHalfEdge());
                    }
                }

                // Create new faces using the edgeloop
                for (size_t i = 0; i < horizonEdgeCount; i++)
                {
                    const size_t AB = m_horizonEdges[i];

                    auto horizonEdgeVertexIndices = m_mesh.getVertexIndicesOfHalfEdge(m_mesh.m_halfEdges[AB]);
                    size_t A, B, C;
                    A = horizonEdgeVertexIndices[0];
                    B = horizonEdgeVertexIndices[1];
                    C = activePointIndex;

                    const size_t newFaceIndex = m_mesh.addFace();
                    m_newFaceIndices.push_back(newFaceIndex);

                    const size_t CA = m_newHalfEdgeIndices[2 * i + 0];
                    const size_t BC = m_newHalfEdgeIndices[2 * i + 1];

                    m_mesh.m_halfEdges[AB].m_next = BC;
                    m_mesh.m_halfEdges[BC].m_next = CA;
                    m_mesh.m_halfEdges[CA].m_next = AB;

                    m_mesh.m_halfEdges[BC].m_face = newFaceIndex;
                    m_mesh.m_halfEdges[CA].m_face = newFaceIndex;
                    m_mesh.m_halfEdges[AB].m_face = newFaceIndex;

                    m_mesh.m_halfEdges[CA].m_endVertex = A;
                    m_mesh.m_halfEdges[BC].m_endVertex = C;

                    auto &newFace = m_mesh.m_faces[newFaceIndex];

                    const Vector3<T> planeNormal = mathutils::getTriangleNormal(m_vertexData[A], m_vertexData[B], activePoint);
                    newFace.m_P = Plane<T>(planeNormal, activePoint);
                    newFace.m_he = AB;

                    m_mesh.m_halfEdges[CA].m_opp = m_newHalfEdgeIndices[i > 0 ? i * 2 - 1 : 2 * horizonEdgeCount - 1];
                    m_mesh.m_halfEdges[BC].m_opp = m_newHalfEdgeIndices[((i + 1) * 2) % (horizonEdgeCount * 2)];
                }

                // Assign points that were on the positive side
                // of the disabled faces to the new faces.
                for (auto &disabledPoints : m_disabledFacePointVectors)
                {
                    assert(disabledPoints);
                    for (const auto &point : *(disabledPoints))
                    {
                        if (point == activePointIndex)
                        {
                            continue;
                        }
                        for (size_t j = 0; j < horizonEdgeCount; j++)
                        {
                            if (addPointToFace(m_mesh.m_faces[m_newFaceIndices[j]], point))
                            {
                                break;
                            }
                        }
                    }
                    // The points are no longer needed: we can move them to the vector pool for reuse.
                    reclaimToIndexVectorPool(disabledPoints);
                }

                // Increase face stack size if needed
                for (const auto newFaceIndex : m_newFaceIndices)
                {
                    auto &newFace = m_mesh.m_faces[newFaceIndex];
                    if (newFace.m_pointsOnPositiveSide)
                    {
                        assert(newFace.m_pointsOnPositiveSide->size() > 0);
                        if (!newFace.m_inFaceStack)
                        {
                            m_faceList.push_back(newFaceIndex);
                            newFace.m_inFaceStack = 1;
                        }
                    }
                }
            }

            // Cleanup
            m_indexVectorPool.clear();
        }

        // Constructs the convex hull into a MeshBuilder object
        // which can be converted to a ConvexHull or Mesh object
        inline void buildMesh(const VertexDataSource<T> &pointCloud, bool CCW, bool useOriginalIndices, T eps)
        {
            // CCW is unused for now
            (void)CCW;
            // useOriginalIndices is unused for now
            (void)useOriginalIndices;

            if (pointCloud.size() == 0)
            {
                m_mesh = MeshBuilder<T>();
                return;
            }
            m_vertexData = pointCloud;

            // Very first: find extreme values and use them
            // to compute the scale of the point cloud.
            m_extremeValues = getExtremeValues();
            m_scale = getScale(m_extremeValues);

            // Epsilon we use depends on the scale
            m_epsilon = eps * m_scale;
            m_epsilonSquared = m_epsilon * m_epsilon;

            // Reset diagnostics
            m_diagnostics = DiagnosticsData();

            // The planar case happens when all the points
            // appear to lie on a two dimensional subspace of R^3.
            m_planar = false;
            createConvexHalfEdgeMesh();
            if (m_planar)
            {
                const size_t extraPointIndex = m_planarPointCloudTemp.size() - 1;
                for (auto &he : m_mesh.m_halfEdges)
                {
                    if (he.m_endVertex == extraPointIndex)
                    {
                        he.m_endVertex = 0;
                    }
                }
                m_vertexData = pointCloud;
                m_planarPointCloudTemp.clear();
            }
        }

        // The public getConvexHull functions will
        // setup a VertexDataSource object and call this
        inline ConvexHull<T> getConvexHull(const VertexDataSource<T> &pointCloud, bool CCW, bool useOriginalIndices, T eps)
        {
            buildMesh(pointCloud, CCW, useOriginalIndices, eps);
            return ConvexHull<T>(m_mesh, m_vertexData, CCW, useOriginalIndices);
        }

    public:
        // Computes convex hull for a given point cloud.
        // Params:
        //   pointCloud: a vector of of 3D points
        //   CCW: whether the output mesh triangles should have CCW orientation
        //   useOriginalIndices: should the output mesh use same vertex indices
        //                       as the original point cloud. If this is false,
        //                       then we generate a new vertex buffer which contains
        //                       only the vertices that are part of the convex hull.
        //   eps: minimum distance to a plane to consider a point being on
        //        positive of it (for a point cloud with scale 1)
        inline ConvexHull<T> getConvexHull(const std::vector<Vector3<T>> &pointCloud,
                                           bool CCW,
                                           bool useOriginalIndices,
                                           T eps = defaultEps<T>())
        {
            {
                VertexDataSource<T> vertexDataSource(pointCloud);
                return getConvexHull(vertexDataSource, CCW, useOriginalIndices, eps);
            }
        }

        // Computes convex hull for a given point cloud.
        // Params:
        //   vertexData: pointer to the first 3D point of the point cloud
        //   vertexCount: number of vertices in the point cloud
        //   CCW: whether the output mesh triangles should have CCW orientation
        //   useOriginalIndices: should the output mesh use same vertex indices
        //                       as the original point cloud. If this is false,
        //                       then we generate a new vertex buffer which contains
        //                       only the vertices that are part of the convex hull.
        //   eps: minimum distance to a plane to consider a point being on positive
        //        side of it (for a point cloud with scale 1)
        inline ConvexHull<T> getConvexHull(const Vector3<T> *vertexData,
                                           size_t vertexCount,
                                           bool CCW,
                                           bool useOriginalIndices,
                                           T eps = defaultEps<T>())
        {
            {
                VertexDataSource<T> vertexDataSource(vertexData, vertexCount);
                return getConvexHull(vertexDataSource, CCW, useOriginalIndices, eps);
            }
        }

        // Computes convex hull for a given point cloud. This function
        // assumes that the vertex data resides in memory
        // in the following format: x_0,y_0,z_0,x_1,y_1,z_1,...
        // Params:
        //   vertexData: pointer to the X component of the first point of the point cloud.
        //   vertexCount: number of vertices in the point cloud
        //   CCW: whether the output mesh triangles should have CCW orientation
        //   useOriginalIndices: should the output mesh use same vertex indices as the
        //                       original point cloud. If this is false, then we generate
        //                       a new vertex buffer which contains only the vertices that
        //                       are part of the convex hull.
        //   eps: minimum distance to a plane to consider a point being on positive side
        //        of it (for a point cloud with scale 1)
        inline ConvexHull<T> getConvexHull(const T *vertexData,
                                           size_t vertexCount,
                                           bool CCW,
                                           bool useOriginalIndices,
                                           T eps = defaultEps<T>())
        {
            VertexDataSource<T> vertexDataSource((const vec3 *)vertexData, vertexCount);
            return getConvexHull(vertexDataSource, CCW, useOriginalIndices, eps);
        }

        // Computes convex hull for a given point cloud. This function assumes
        // that the vertex data resides in memory
        // in the following format: x_0,y_0,z_0,x_1,y_1,z_1,...
        // Params:
        //   vertexData: pointer to the X component of the first point of the point cloud.
        //   vertexCount: number of vertices in the point cloud
        //   CCW: whether the output mesh triangles should have CCW orientation
        //   eps: minimum distance to a plane to consider a point being on positive side
        //        of it (for a point cloud with scale 1)
        // Returns:
        //   Convex hull of the point cloud as a mesh object with half edge structure.
        inline HalfEdgeMesh<T, size_t> getConvexHullAsMesh(const T *vertexData,
                                                           size_t vertexCount,
                                                           bool CCW,
                                                           T eps = defaultEps<T>())
        {
            VertexDataSource<T> vertexDataSource((const vec3 *)vertexData, vertexCount);
            buildMesh(vertexDataSource, CCW, false, eps);
            return HalfEdgeMesh<T, size_t>(m_mesh, m_vertexData);
        }

        // Get diagnostics about last generated convex hull
        inline const DiagnosticsData &getDiagnostics()
        {
            return m_diagnostics;
        }
    };

    // Inline function definitions
    template <typename T>
    inline std::unique_ptr<std::vector<size_t>> QuickHull<T>::getIndexVectorFromPool()
    {
        auto r = std::move(m_indexVectorPool.get());
        r->clear();
        return r;
    }

    template <typename T>
    inline void QuickHull<T>::reclaimToIndexVectorPool(std::unique_ptr<std::vector<size_t>> &ptr)
    {
        const size_t oldSize = ptr->size();
        if ((oldSize + 1) * 128 < ptr->capacity())
        {
            // Reduce memory usage! Huge vectors are needed at the beginning of iteration
            // when faces have many points on their positive side. Later on, smaller vectors
            // will suffice.
            ptr.reset(nullptr);
            return;
        }
        m_indexVectorPool.reclaim(ptr);
    }

    template <typename T>
    inline bool QuickHull<T>::addPointToFace(typename MeshBuilder<T>::Face &f, size_t pointIndex)
    {
        const T D = mathutils::getSignedDistanceToPlane(m_vertexData[pointIndex], f.m_P);
        if (D > 0 && D * D > m_epsilonSquared * f.m_P.m_sqrNLength)
        {
            if (!f.m_pointsOnPositiveSide)
            {
                f.m_pointsOnPositiveSide = std::move(getIndexVectorFromPool());
            }
            f.m_pointsOnPositiveSide->push_back(pointIndex);
            if (D > f.m_mostDistantPointDist)
            {
                f.m_mostDistantPointDist = D;
                f.m_mostDistantPoint = pointIndex;
            }
            return true;
        }
        return false;
    }

} // namespace quickhull

#endif