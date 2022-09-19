#include "occutils/Wire.hxx"
#include "occutils/Edge.hxx"
#include "occutils/Pipe.hxx"
#include "occutils/Face.hxx"
#include "occutils/Point.hxx"
#include "occutils/Equality.hxx"
#include <BRepLib_MakeWire.hxx>
#include <BRepLib_MakeWire.hxx>
#include <stdexcept>
#include <BRepTools_WireExplorer.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <Geom_Line.hxx>
#include <Geom_Circle.hxx>
#include <Geom_Ellipse.hxx>
#include <Geom_Parabola.hxx>
#include <Geom_Line.hxx>
#include <Geom_Hyperbola.hxx>
#include <Geom_TrimmedCurve.hxx>
#include <Geom_OffsetCurve.hxx>
#include <Geom_BezierCurve.hxx>
#include <Geom_BSplineCurve.hxx>
#include <TopAbs_Orientation.hxx>

using namespace OCCUtils;

TopoDS_Wire OCCUtils::Wire::FromEdges(const std::initializer_list<TopoDS_Edge>& edges) {
    BRepLib_MakeWire wireMaker;
    for(const auto& edge : edges) {
        if(edge.IsNull()) {
            continue;
        }
        wireMaker.Add(edge);
    }
    return wireMaker.IsDone() ? wireMaker.Wire() : TopoDS_Wire();
}

TopoDS_Wire OCCUtils::Wire::FromEdges(const std::vector<TopoDS_Edge>& edges) {
    BRepLib_MakeWire wireMaker;
    for(const auto& edge : edges) {
        if(edge.IsNull()) {
            continue;
        }
        wireMaker.Add(edge);
    }
    return wireMaker.IsDone() ? wireMaker.Wire() : TopoDS_Wire();
}

OCCUtils::Wire::IncrementalBuilder::IncrementalBuilder(const gp_Pnt& pnt)
    : current(pnt), currentDirection(std::nullopt), edges() {
    edges.reserve(25); // Prevent frequent reallocations at the expense of some memory
}

/**
 * Add a line segment
 */
void OCCUtils::Wire::IncrementalBuilder::Line(double dx, double dy, double dz) {
    gp_Pnt p1(current); // Copy current point
    // Increment coordinates
    current = current + gp_Pnt(dx, dy, dz);
    // Create edges
    currentDirection = gp_Vec(p1, current);
    edges.emplace_back(Edge::FromPoints(p1, current));
}


void OCCUtils::Wire::IncrementalBuilder::Arc90(
    double dx, double dy, double dz,
    double centerDx, double centerDy, double centerDz,
    const gp_Dir& normal) {
        gp_Pnt p2 = current + gp_Pnt(dx, dy, dz);
        gp_Pnt center = current + gp_Pnt(centerDx, centerDy, centerDz);
        gp_Dir resultingDirection(gp_Vec(current, center));
        double radius = current.Distance(center);
        double radiusAlt = p2.Distance(center);
        if(abs(radius - radiusAlt) >= Precision::Confusion()) {
            throw std::invalid_argument("dx/dy/dz does not match centerD...!");
        }
        // Current algorithm: Compute both options,
        // one is 90° and one is 270°, select the shorter one. 
        auto option1 = Edge::CircleSegment(gp_Ax2(center, normal), radius, current, p2);
        auto option2 = Edge::CircleSegment(gp_Ax2(center, normal), radius, p2, current);
        double length1 = Edge::Length(option1);
        double length2 = Edge::Length(option2);
        edges.emplace_back(length1 < length2 ? option1 : option2);
        current = p2;
        currentDirection = resultingDirection;
    }

TopoDS_Wire OCCUtils::Wire::IncrementalBuilder::Wire() {
    return Wire::FromEdges(edges);
}

gp_Pnt OCCUtils::Wire::IncrementalBuilder::Location() {
    return gp_Pnt(current); // make copy to avoid modification
}

/**
 * Create a pipe from the wire using the given profile.
 */
TopoDS_Shape OCCUtils::Wire::IncrementalBuilder::Pipe(const TopoDS_Face& profile) {
    return Pipe::FromSplineAndProfile(this->Wire(), profile);
}


TopoDS_Shape OCCUtils::Wire::IncrementalBuilder::PipeWithCircularProfile(double radius) {
    auto profile = Face::FromEdge(Edge::FullCircle(gp_Ax2(current, currentDirection.value_or(Direction::Z())), radius));
    return Pipe(profile);
}

std::optional<gp_Dir> OCCUtils::Wire::IncrementalBuilder::Direction() {
    // Make copy of direction to prevent modification!
    if(currentDirection.has_value()) {
        return std::make_optional(gp_Dir(currentDirection.value()));
    } else {
        return std::nullopt;
    }
}

TopoDS_Wire OCCUtils::Wire::FromPoints(const std::vector<gp_Pnt>& points, bool close) {
    if (points.size() < 2) {
        return TopoDS_Wire ();
    }
    // Build directly without making a vector of edges
    // This is likely slightly more efficient
    BRepLib_MakeWire makeWire;
    for (size_t i = 0; i < points.size() - 1; i++)
    {
        const auto& p1 = points[i];
        const auto& p2 = points[i + 1];
        // Ignore duplicate points
        if (p1 == p2) {
            continue;
        }
        makeWire.Add(Edge::FromPoints(p1, p2));
    }
    // Close curve if enabled
    if(close) {
        const auto& p0 = points[0];
        const auto& plast = points[points.size() - 1];
        if(p0 != plast) {
            makeWire.Add(Edge::FromPoints(p0, plast));
        }
    }
    return makeWire.Wire ();
}

gp_Dir OCCUtils::Wire::NormalDir(const TopoDS_Wire& wire)
{
	double x = 0, y = 0, z = 0;
	gp_Pnt currentStart, previousEnd, first;
	int count = 0;
	TopLoc_Location loc;
	Standard_Real start, end;

	for (BRepTools_WireExplorer wEx(wire); wEx.More(); wEx.Next())
	{
		const TopoDS_Vertex& v = wEx.CurrentVertex();
		currentStart = BRep_Tool::Pnt(v);
		Handle(Geom_Curve) c3d = BRep_Tool::Curve(wEx.Current(), loc, start, end);
		if (!c3d.IsNull())
		{
			Handle(Geom_Curve) c3dptr = Handle(Geom_Curve)::DownCast(c3d->Transformed(loc.Transformation()));
			Handle(Standard_Type) cType = c3dptr->DynamicType();
			if (cType == STANDARD_TYPE(Geom_Line))
			{
				if (count > 0)
					AddNewellPoint(previousEnd, currentStart, x, y, z);
				else
					first = currentStart;
				previousEnd = currentStart;
			}
			else if (wEx.Current().Closed() && ((cType == STANDARD_TYPE(Geom_Circle)) ||
				(cType == STANDARD_TYPE(Geom_Ellipse)) ||
				(cType == STANDARD_TYPE(Geom_Parabola)) ||
				(cType == STANDARD_TYPE(Geom_Hyperbola)))) //it is a conic
			{
				Handle(Geom_Conic) conic = Handle(Geom_Conic)::DownCast(c3dptr);
				return conic->Axis().Direction();

			}
			else if ((cType == STANDARD_TYPE(Geom_Circle)) ||
				(cType == STANDARD_TYPE(Geom_Ellipse)) ||
				(cType == STANDARD_TYPE(Geom_Parabola)) ||
				(cType == STANDARD_TYPE(Geom_Hyperbola)) ||
				(cType == STANDARD_TYPE(Geom_TrimmedCurve)) ||
				(cType == STANDARD_TYPE(Geom_OffsetCurve)) ||
				(cType == STANDARD_TYPE(Geom_BezierCurve)) ||
				(cType == STANDARD_TYPE(Geom_BSplineCurve)))
			{
				// we identify the Us of quadrant points along the curve to compute the normal
				//
				BRepAdaptor_Curve curve(wEx.Current());
				TopAbs_Orientation ors = wEx.Current().Orientation();
				double uStart = curve.FirstParameter();
				double uEnd = curve.LastParameter();
				double u0, u1, u2, u3;
				double delta;
				if (ors != TopAbs_REVERSED)
				{
					u0 = uStart; // start from the start
					delta = (uEnd - uStart) / 4; // and go forward a bit for each step
				}
				else
				{
					u0 = uEnd; // start from the end
					delta = (uEnd - uStart) / -4; // and go back a bit for each step
				}
				u1 = u0 + delta;
				u2 = u1 + delta;
				u3 = u2 + delta;

				// then we get the points
				gp_Pnt p0; gp_Pnt p1; gp_Pnt p2; gp_Pnt p3;
				curve.D0(u0, p0);
				curve.D0(u1, p1);
				curve.D0(u2, p2);
				curve.D0(u3, p3);

				// then add the points to the newell evaluation
				if (count > 0)
				{
					AddNewellPoint(previousEnd, p0, x, y, z);
					AddNewellPoint(p0, p1, x, y, z);
					AddNewellPoint(p1, p2, x, y, z);
					AddNewellPoint(p2, p3, x, y, z);
					previousEnd = p3;
				}
				else
				{
					first = p0;
					AddNewellPoint(first, p1, x, y, z);
					AddNewellPoint(p1, p2, x, y, z);
					AddNewellPoint(p2, p3, x, y, z);
					previousEnd = p3;
				}
			}
			else //throw AN EXCEPTION
			{
				throw Standard_Failure("Unsupported Edge type");
			}
		}
		count++;
	}
	//do the last one
	AddNewellPoint(previousEnd, first, x, y, z);
	gp_Dir dir(x, y, z);
	return dir;
}

void OCCUtils::Wire::AddNewellPoint(const gp_Pnt& previous, const gp_Pnt& current, double& x, double& y, double& z)
{
	const double& xn = previous.X();
	const double& yn = previous.Y();
	const double& zn = previous.Z();
	const double& xn1 = current.X();
	const double& yn1 = current.Y();
	const double& zn1 = current.Z();

	x += (yn - yn1) * (zn + zn1);
	y += (xn + xn1) * (zn - zn1);
	z += (xn - xn1) * (yn + yn1);
}
