// OCCTExample.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include <iostream>
#include <RWObj.hxx>
#include <TopoDS.hxx>
#include <Geom_Line.hxx>
#include <Geom_Plane.hxx>
#include <TopoDS_Face.hxx>
#include <BRep_Builder.hxx>
#include <GC_MakePlane.hxx>
#include <GeomAdaptor_Surface.hxx>
#include <GeomLib_IsPlanarSurface.hxx>
#include <BRepBuilderAPI_Sewing.hxx>
#include <ProjLib_ProjectOnPlane.hxx>
#include <IGESControl_Reader.hxx>
#include <BRepAlgoAPI_Section.hxx>
#include <IGESControl_Controller.hxx> 
#include <IGESControl_Writer.hxx> 
#include <BRepIntCurveSurface_Inter.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <occutils/Surface.hxx>
#include <occutils/Face.hxx>
#include <occutils/Plane.hxx>
#include <occutils/Edge.hxx>
#include <occutils/Curve.hxx>
#include <TopExp_Explorer.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include "FollowSlope/FollowSlopeService.h"

void FolloeSlopeDemoByDONG()
{
    IGESControl_Reader reader;
    IFSelect_ReturnStatus stat = reader.ReadFile("D:\\beam.igs");
    Handle(TColStd_HSequenceOfTransient)  list = reader.GiveList();
    Standard_Integer nbtrans = reader.TransferList(list);
    //TopoDS_Shape shape = reader.Shape();
    //TopoDS_Edge edge = TopoDS::Edge(shape);
    //TopAbs_ShapeEnum type = shape.ShapeType();
    //for (TopExp_Explorer expVert(edge, TopAbs_VERTEX); expVert.More(); expVert.Next())
    //{
    //    TopoDS_Vertex vert = TopoDS::Vertex(expVert.Current());
    //    gp_Pnt v = BRep_Tool::Pnt(vert);
    //}

    IGESControl_Controller::Init();
    IGESControl_Writer ICW("MM", 0);

    IGESControl_Reader reader2;
    IFSelect_ReturnStatus stat2 = reader2.ReadFile("D:\\sloperoof.igs");
    Handle(TColStd_HSequenceOfTransient) list2 = reader2.GiveList();
    Standard_Integer nbtrans2 = reader2.TransferList(list2);
    for (Standard_Integer i = 1; i <= reader2.NbShapes(); i++)
    {
        TopoDS_Shape shape = reader2.Shape(i);
        TopoDS_Face face = TopoDS::Face(shape);
        double area = OCCUtils::Surface::Area(face);
        Handle(Geom_Surface) surface = BRep_Tool::Surface(face);
        GeomLib_IsPlanarSurface isPlanar(surface);
        if (isPlanar.IsPlanar())
        {
            Handle(Geom_Plane) plane = new Geom_Plane(isPlanar.Plan());
            ProjLib_ProjectOnPlane project(plane->Pln().Position(), gp_Dir(0, 0, 1));
            for (Standard_Integer i = 1; i <= reader.NbShapes(); i++)
            {
                TopoDS_Shape shape = reader.Shape(i);
                std::vector<gp_Pnt> vertices;
                std::vector<IntCurveSurface_IntersectionPoint> intersections;
                for (TopExp_Explorer expVert(TopoDS::Edge(shape), TopAbs_VERTEX); expVert.More(); expVert.Next())
                {
                    TopoDS_Vertex vert = TopoDS::Vertex(expVert.Current());
                    vertices.push_back(BRep_Tool::Pnt(vert));
                    //gp_Dir dir(0, 0, 1);
                    //BRepIntCurveSurface_Inter inter;
                    //inter.Init(face, gp_Lin(BRep_Tool::Pnt(vert), dir), Precision::Confusion());
                    //for (; inter.More(); inter.Next())
                    //{
                    //    intersections.push_back(inter.Point());
                    //}
                }

                TopoDS_Edge beam = OCCUtils::Edge::FromPoints(vertices[0], vertices[1]);

                Standard_Real umin, umax;
                // Get unbounded curve plus separate bounding parameters
                Handle(Geom_Curve) rawCurve = BRep_Tool::Curve(beam, umin, umax);
                Handle(GeomAdaptor_Curve) adaptor = new GeomAdaptor_Curve(rawCurve, umin, umax);
                project.Load(adaptor, Precision::Confusion());
                Handle(GeomAdaptor_Curve) result = project.GetResult();
                Handle(Geom_TrimmedCurve) trimmed = new Geom_TrimmedCurve(result->Curve(), umin, umax);
                BRepBuilderAPI_MakePolygon maker(vertices[0], vertices[1],
                    trimmed->EndPoint(), trimmed->StartPoint(), Standard_True);
                BRepBuilderAPI_MakeFace faceMaker(maker.Wire());
                double area = OCCUtils::Surface::Area(faceMaker.Face());
                BRepAlgoAPI_Section section(face, faceMaker.Face());
                for (TopExp_Explorer explorer(section.Shape(), TopAbs_EDGE); explorer.More(); explorer.Next())
                {
                    TopoDS_Shape shape = explorer.Value();
                    //
                    ICW.AddShape(shape);
                }
            }
        }
    }


    ICW.ComputeModel();
    ICW.Write("D:\\result.igs");
    //Handle(Poly_Triangulation) aTriangulation = RWObj::ReadFile("D:\\sloperoof.obj");
    //if (!aTriangulation.IsNull() && aTriangulation->HasGeometry())
    //{
    //    // https://dev.opencascade.org/content/build-topodsshape-triangulation
    //    std::vector<TopoDS_Face> faces;
    //    std::vector<Handle(Geom_Plane)> planes;
    //    for (Standard_Integer i = 1; i <= aTriangulation->NbTriangles(); i++)
    //    {
    //        Poly_Triangle triangle = aTriangulation->Triangle(i);
    //        std::vector<gp_Pnt> points = {
    //            aTriangulation->Node(triangle.Value(1)),
    //            aTriangulation->Node(triangle.Value(2)),
    //            aTriangulation->Node(triangle.Value(3)),
    //        };
    //        TopoDS_Face face = OCCUtils::Face::FromPoints(points);
    //        double area = OCCUtils::Surface::Area(face);
    //        Handle(Geom_Surface) surface = BRep_Tool::Surface(face);
    //        GeomLib_IsPlanarSurface isPlanar(surface);
    //        if (isPlanar.IsPlanar())
    //        {
    //            Handle(Geom_Plane) plane = new Geom_Plane(isPlanar.Plan());
    //            ProjLib_ProjectOnPlane project(plane->Pln().Position(), gp_Dir(0, 0, 1));
    //            for (Standard_Integer i = 1; i <= reader.NbShapes(); i++)
    //            {
    //                std::vector<gp_Pnt> vertices;
    //                TopoDS_Shape shape = reader.Shape(i);
    //                for (TopExp_Explorer expVert(TopoDS::Edge(shape), TopAbs_VERTEX); expVert.More(); expVert.Next())
    //                {
    //                    TopoDS_Vertex vert = TopoDS::Vertex(expVert.Current());
    //                    vertices.push_back(BRep_Tool::Pnt(vert));
    //                }
    //                TopoDS_Edge beam = OCCUtils::Edge::FromPoints(vertices[0], vertices[1]);

    //                Standard_Real umin, umax;
    //                // Get unbounded curve plus separate bounding parameters
    //                Handle(Geom_Curve) rawCurve = BRep_Tool::Curve(beam, umin, umax);
    //                Handle(GeomAdaptor_Curve) adaptor = new GeomAdaptor_Curve(rawCurve, umin, umax);
    //                project.Load(adaptor, Precision::Confusion());
    //                Handle(GeomAdaptor_Curve) result = project.GetResult();
    //                Handle(Geom_TrimmedCurve) trimmed = new Geom_TrimmedCurve(result->Curve(), umin, umax);
    //                //Handle(Geom_Curve) curve = adaptor.Curve();


                    //topods_shape shape = reader.shape(i);
                    //topods_edge edge = topods::edge(shape);
                    //for (topexp_explorer expvert(edge, topabs_vertex); expvert.more(); expvert.next())
                    //{
                    //    topods_vertex vert = topods::vertex(expvert.current());

                    //    // brepintcurvesurface_inter 
                    //    // intcurvesface_shapeintersector
                    //    gp_dir dir = gp_dir(0, 0, 1);
                    //    gp_pnt v = brep_tool::pnt(vert);
                    //    brepintcurvesurface_inter inter;
                    //    inter.init(face, gp_lin(v, dir), precision::confusion());
                    //    for (; inter.more(); inter.next())
                    //    {
                    //        intcurvesurface_intersectionpoint pt = inter.point();
                    //    }
                    //}
    //            }
    //        }
    //    };

    //    {
    //        TopoDS_Edge edge = OCCUtils::Edge::FromPoints(gp_Pnt(0, 0, 0), gp_Pnt(0, 1111, 0));
    //        GeomAdaptor_Curve adaptor = OCCUtils::Curve::FromEdge(edge);
    //        Handle(Geom_Curve) curve = adaptor.Curve();
    //        Handle(Geom_TrimmedCurve) trimmed = Handle(Geom_TrimmedCurve)::DownCast(curve);
    //    }

    //    //TopoDS::
    //    //TopExp_Explorer::
    //    //GeomProjLib::ProjectOnPlane

    //    //BRepBuilderAPI_Sewing sewing;
    //    //for (auto iter = faces.begin(); iter != faces.end(); ++iter)
    //    //{
    //    //    sewing.Add(*iter);
    //    //};
    //    //sewing.Perform();
    //    //const TopoDS_Shape& shape = sewing.SewedShape();
    //}
}

int main()
{
    //FolloeSlopeDemoByDONG();
    FollowSlopeService followService;
    followService.FollowSlope("D:\\beam1.igs", "D:\\roof1.igs");
    return 0;
}

