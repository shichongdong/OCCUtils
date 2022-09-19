#pragma once
#include <string>
#include <IGESControl_Reader.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS.hxx>
#include <BRep_Tool.hxx>
#include <GeomLib_IsPlanarSurface.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS_Vertex.hxx>
#include <TopoDS_Wire.hxx>
#include <BRepTools.hxx>
#include <TopoDS_Edge.hxx>
#include <vector>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepAlgoAPI_Section.hxx>
#include <IGESControl_Writer.hxx>
#include <occutils/Point.hxx>
#include <occutils/Direction.hxx>
#include <occutils/Edge.hxx>
#include <occutils/Face.hxx>
#include <occutils/Solid.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <map>
#include <Geom_Plane.hxx>
#include <BRepBuilderAPI_MakeSolid.hxx>
#include <BRepPrim_Builder.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <BRepBuilderAPI_Sewing.hxx>

class FollowSlopeService
{
public :
    void FollowSlope(std::string beamPath, std::string roofPath);
    std::vector<std::pair<TopoDS_Edge, gp_Dir>> CreateCompleteBeam(std::vector<std::pair<TopoDS_Edge, gp_Dir>>);
    TopoDS_Edge MergeBeamLines(std::vector<TopoDS_Edge> edges, gp_Pnt& pt1, gp_Pnt& pt2);
    std::vector<TopoDS_Edge> CreateBeamSolidEdges(std::vector<std::pair<TopoDS_Edge, gp_Dir>>);
    std::vector<TopoDS_Face> CreateBeamSolidSolids(std::vector<std::pair<TopoDS_Edge, gp_Dir>>);
    bool CalFaceDirection(TopoDS_Face, TopoDS_Edge, gp_Dir& resDir);
};