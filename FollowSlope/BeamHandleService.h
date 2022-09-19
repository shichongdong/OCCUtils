#pragma once
#include <vector>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <occutils/Face.hxx>
#include <occutils/Edge.hxx>
#include <occutils/Point.hxx>
#include <occutils/Surface.hxx>

class BeamHandleService
{
public :
    std::vector<TopoDS_Edge> CreateBeamLine(std::vector<TopoDS_Face>);
    std::vector<std::vector<TopoDS_Face>> ClassifyTriangle(std::vector<TopoDS_Face>);
    gp_Dir CalFaceDirection(TopoDS_Face face);
    std::vector<std::vector<TopoDS_Face>> FilterFaceByNormal(std::vector<std::vector<TopoDS_Face>> faceVec);
    std::vector<TopoDS_Face> GetNeedBeamFace(std::vector<std::vector<TopoDS_Face>> faceVec);
    std::vector<TopoDS_Edge> FacePoint(std::vector<TopoDS_Face> faces);
};

