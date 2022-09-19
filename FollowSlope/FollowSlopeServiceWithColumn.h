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

class FollowSlopeServiceWithColumn
{
public:
    void FollowSlope(std::string beamPath, std::string roofPath, std::string columnPath);
private:
    std::vector<TopoDS_Edge> GetBeamEdge(IGESControl_Reader);
    std::vector<TopoDS_Edge> GetColumnEdge(IGESControl_Reader);
    std::vector<TopoDS_Face> BeamFollowSlope(std::vector<TopoDS_Edge> beamEdges, double maxZValue, IGESControl_Reader roofReader);
    std::vector<TopoDS_Face> ColumnFollowSlope(std::vector <std::vector<TopoDS_Edge>> columnEdges, double maxZValue, IGESControl_Reader roofReader);
    TopoDS_Face CreateColumnSolidFaces(std::vector<TopoDS_Edge>, TopoDS_Edge);
    TopoDS_Face CreateColumnTopSolidFaces(std::vector<TopoDS_Edge>);
    void getFiles(std::string path, std::vector<std::string>& files);
    std::vector<std::pair<TopoDS_Edge, gp_Dir>> CreateCompleteBeam(std::vector<std::pair<TopoDS_Edge, gp_Dir>>, gp_Pnt pt1, gp_Pnt pt2);
};

