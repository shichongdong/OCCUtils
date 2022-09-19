#pragma once
#include <vector>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <occutils/Face.hxx>
#include <occutils/Edge.hxx>
#include <occutils/Point.hxx>
#include <occutils/Surface.hxx>

class ColumnHandleService
{
public:
    std::vector<TopoDS_Edge> CreateColumnLine(std::vector<TopoDS_Face>);
private:
    std::vector<std::vector<TopoDS_Face>> ClassifyTriangle(std::vector<TopoDS_Face>);
    std::vector<TopoDS_Face> GetNeedColumnFace(std::vector<std::vector<TopoDS_Face>> faceVec);
    std::vector<TopoDS_Edge> FaceEdges(std::vector<TopoDS_Face> faces);
};

