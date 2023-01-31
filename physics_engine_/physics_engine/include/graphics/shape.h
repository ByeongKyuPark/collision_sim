#pragma once

#include <vector>
#include <opengl/glm/glm.hpp>
#include <memory>//shared_ptr
#include <physics/Vector.h>

namespace Graphics
{
    using physics::Vector;
    using std::shared_ptr;
    using std::make_shared;

    struct Vertex
    {
        Vertex(glm::vec3 pos, glm::vec3 nrm, glm::vec2 uv) : pos(pos), nrm(nrm), uv(uv) {}

        Vertex() :Vertex{ glm::vec3{},glm::vec3{},glm::vec3{} } {}

        glm::vec3 pos;
        glm::vec3 nrm;
        glm::vec2 uv;
    };

    using VertexBufferType = std::vector<Vertex>;
    using FrameIndexBufferType = std::vector<unsigned>;
    using IndexBufferType = std::vector<unsigned>;

    struct VertexLayout
    {
        int location;
        int size;
        int offset;
    };

    constexpr VertexLayout vertexLayout[] =
    {
        { 0, 3, offsetof(Vertex, pos) },
        { 1, 3, offsetof(Vertex, nrm)},
        { 2, 2, offsetof(Vertex, uv)}
    };
    constexpr int LayoutSize = sizeof(VertexLayout);
    constexpr int NumAttribs = sizeof(vertexLayout) / LayoutSize;      //pos only for now
    
    class Shape
    {
    public:
        friend class Renderer;

    protected:
        VertexBufferType vertexBuffer;
        shared_ptr<IndexBufferType> indexBuffer;
        shared_ptr<FrameIndexBufferType> frameIndexBuffer;

        unsigned int polygonVAO;
        unsigned int frameVAO;

    public:
        Shape(shared_ptr<IndexBufferType> IdxBuffer=nullptr, shared_ptr<FrameIndexBufferType> FrameIdxBuffer=nullptr)
            :indexBuffer{ IdxBuffer }, frameIndexBuffer{ FrameIdxBuffer }
            , polygonVAO{}, frameVAO{}
        {}
        virtual ~Shape() {}

        void CreateVertexArray();
        virtual void CreateVertices(const Vector& scale) = 0;
    };

    class Box : public Shape
    {
        static shared_ptr<IndexBufferType> boxIndexBuffer;
        static shared_ptr<FrameIndexBufferType> boxFrameIndexBuffer;
    public:
        Box(const Vector& scale = {1.f,1.f,1.f});
        void CreateVertices(const Vector& vec = {1.f,1.f,1.f}) override final;
    };

    class Sphere : public Shape
    {
        static shared_ptr<IndexBufferType> sphereIndexBuffer;
        static shared_ptr<FrameIndexBufferType> sphereFrameIndexBuffer;

    public:
        static const int SECTOR_CNT = 36;
        static const int STACK_CNT = 18;

    public:
        Sphere(const Vector& scale = {1.f,1.f,1.f});
        void CreateVertices(const Vector& scale={1.f,1.f,1.f}) override final;
        void CreateIndices();
    };
}
