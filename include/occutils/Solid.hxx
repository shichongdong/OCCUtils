#pragma once
#include <TopoDS_Shape.hxx>
#include <TopoDS_Face.hxx>
#include <occutils/Face.hxx>
#include <occutils/Wire.hxx>
#include <TopoDS_Vertex.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_VertexInspector.hxx>
#include <BRep_Builder.hxx>
#include <TopTools_SequenceOfShape.hxx>
#include <TopTools_DataMapOfShapeListOfShape.hxx>
#include <TopoDS.hxx>
#include <TopExp.hxx>
#include <ShapeAnalysis.hxx>
#include <BRepCheck_Shell.hxx>
#include <ShapeFix_Shell.hxx>
#include <ShapeFix_Shape.hxx>
#include <TopExp_Explorer.hxx>
#include <BRepBuilderAPI_MakeSolid.hxx>
#include <BRepClass3d_SolidClassifier.hxx>
#include <BRepPrim_Builder.hxx>
#include <BRepCheck_Solid.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <vector>

namespace OCCUtils {
    namespace Solid {
        TopoDS_Shape CreateSolidShapeByFaces(std::vector<TopoDS_Face>);

        std::vector<TopoDS_Solid> CreateSolidShapeByShell(std::vector<TopoDS_Face>);

        std::vector<TopoDS_Solid> CreateSolidByFaces(std::vector<TopoDS_Face>);

        std::vector<TopoDS_Solid> CreateSolidWithVoids(std::vector<TopoDS_Face>);

        TopoDS_Compound MakeCompound(std::vector<TopoDS_Face>);

        TopoDS_Shell MakeShell(std::vector<TopoDS_Face>);

        int Count(TopoDS_Compound*);
    }
}