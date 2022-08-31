// OCCTExample.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include <iostream>
#include <RWObj.hxx>
#include <TopoDS.hxx>
#include <Geom_Plane.hxx>
#include <TopoDS_Face.hxx>
#include <BRep_Builder.hxx>
#include <GeomAdaptor_Surface.hxx>
#include <GeomLib_IsPlanarSurface.hxx>
#include <BRepBuilderAPI_Sewing.hxx>
#include <ProjLib_ProjectOnPlane.hxx>
#include <BRepIntCurveSurface_Inter.hxx>
#include <occutils/Surface.hxx>
#include <occutils/Face.hxx>
#include <occutils/Plane.hxx>
#include <occutils/Edge.hxx>
#include <occutils/Curve.hxx>

int main()
{
    Handle(Poly_Triangulation) aTriangulation = RWObj::ReadFile("D:\\sloperoof.obj");
    if (!aTriangulation.IsNull() && aTriangulation->HasGeometry())
    {
        // https://dev.opencascade.org/content/build-topodsshape-triangulation
        std::vector<TopoDS_Face> faces;
        std::vector<Handle(Geom_Plane)> planes;
        for (Standard_Integer i = 1; i <= aTriangulation->NbTriangles(); i++)
        {
            Poly_Triangle triangle = aTriangulation->Triangle(i);
            std::vector<gp_Pnt> points = {
                aTriangulation->Node(triangle.Value(1)),
                aTriangulation->Node(triangle.Value(2)),
                aTriangulation->Node(triangle.Value(3)),
            };
            TopoDS_Face face = OCCUtils::Face::FromPoints(points);
            double area = OCCUtils::Surface::Area(face);
            Handle(Geom_Surface) surface = BRep_Tool::Surface(face);
            GeomLib_IsPlanarSurface isPlanar(surface);
            if (isPlanar.IsPlanar())
            {
                // BRepIntCurveSurface_Inter 
                // IntCurvesFace_ShapeIntersector
                BRepIntCurveSurface_Inter inter;
            }
        };

        {
            TopoDS_Edge edge = OCCUtils::Edge::FromPoints(gp_Pnt(0, 0, 0), gp_Pnt(0, 1111, 0));
            GeomAdaptor_Curve adaptor = OCCUtils::Curve::FromEdge(edge);
            Handle(Geom_Curve) curve = adaptor.Curve();
            Handle(Geom_TrimmedCurve) trimmed = Handle(Geom_TrimmedCurve)::DownCast(curve);
        }

        //TopoDS::
        //TopExp_Explorer::
        //GeomProjLib::ProjectOnPlane

        //BRepBuilderAPI_Sewing sewing;
        //for (auto iter = faces.begin(); iter != faces.end(); ++iter)
        //{
        //    sewing.Add(*iter);
        //};
        //sewing.Perform();
        //const TopoDS_Shape& shape = sewing.SewedShape();
    }
    return 0;
}

