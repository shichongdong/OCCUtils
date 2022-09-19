#include "occutils/Solid.hxx"

TopoDS_Shape OCCUtils::Solid::CreateSolidShapeByFaces(std::vector<TopoDS_Face> faces)
{
    double tolerance = 0.00001;

    BRep_Builder builder;
    TopTools_SequenceOfShape vertices;
	TopTools_DataMapOfShapeListOfShape edgeMap;
	TopoDS_Shell shell;
	builder.MakeShell(shell);
    BRepBuilderAPI_VertexInspector inspector(tolerance);
    NCollection_CellFilter<BRepBuilderAPI_VertexInspector> vertexCellFilter;
    for (TopoDS_Face face : faces)
    {
        TopoDS_Vertex currentTail;
        BRepBuilderAPI_MakeWire wireMaker;
		TopoDS_Wire outerLoop;
		TopTools_SequenceOfShape innerLoops;
        auto pts = OCCUtils::Face::FacePoints(face);
		pts.push_back(pts[0]);
        for (gp_Pnt p : pts)
        {
            inspector.ClearResList();
            inspector.SetCurrent(p.Coord());
            vertexCellFilter.Inspect(p.Coord(), inspector);
            TColStd_ListOfInteger results = inspector.ResInd();
            TopoDS_Vertex vertex;
            if (results.Size() > 0) //hit
            {
                //just take the first one as we don't add vertices more than once to a cell
                int vertexIdx = results.First();
                vertex = TopoDS::Vertex(vertices.Value(vertexIdx));
            }
            else //miss
            {
                inspector.Add(p.Coord());
                //build the vertex

                builder.MakeVertex(vertex, p, tolerance);
                vertices.Append(vertex); //it will have the same index as the point in the inspector
                gp_XYZ coordMin = inspector.Shift(p.Coord(), -tolerance);
                gp_XYZ coordMax = inspector.Shift(p.Coord(), tolerance);
                vertexCellFilter.Add(vertices.Size(), coordMin, coordMax);
            }
			if (currentTail.IsNull()) //first one
			{
				currentTail = vertex;
			}
			else if (!currentTail.IsSame(vertex)) //skip if it the same as the last one
			{
				bool sharedEdge = false;
				//make an edge
				if (edgeMap.IsBound(vertex)) //we have an edge that starts at this ones end, it will need to be reversed
				{
					TopTools_ListOfShape edges = edgeMap.Find(vertex);
					for (auto it = edges.cbegin(); it != edges.cend(); it++)
					{
						TopoDS_Edge edge = TopoDS::Edge(*it);
						TopoDS_Vertex edgeEnd = TopExp::LastVertex(edge, false); //it will laways be forward oriented
						if (edgeEnd.IsSame(currentTail)) //we want this edge reversed
						{
							wireMaker.Add(TopoDS::Edge(edge.Reversed()));
							sharedEdge = true;
							break;
						}
					}
				}
				if (!sharedEdge && edgeMap.IsBound(currentTail)) //we have an edge that starts at this ones end
				{
					TopTools_ListOfShape edges = edgeMap.Find(currentTail);
					for (auto it = edges.cbegin(); it != edges.cend(); it++)
					{
						TopoDS_Edge edge = TopoDS::Edge(*it);
						TopoDS_Vertex edgeEnd = TopExp::LastVertex(edge, false); //it will laways be forward oriented
						if (edgeEnd.IsSame(vertex)) //we want this edge 
						{
							wireMaker.Add(TopoDS::Edge(edge));
							sharedEdge = true;
							break;
						}
					}
				}
				if (!sharedEdge) //make and add the new forward oriented edge if we have not found one
				{
					TopoDS_Edge edge = BRepBuilderAPI_MakeEdge(currentTail, vertex);
					wireMaker.Add(edge);
					if (edgeMap.IsBound(currentTail)) //add it to the list
					{
						edgeMap.ChangeFind(currentTail).Append(edge);
					}
					else //create a new list
					{
						TopTools_ListOfShape edges;
						edges.Append(edge);
						edgeMap.Bind(currentTail, edges);
					}
				}
				currentTail = vertex; //move the tail on
			}	
        }
		if (!wireMaker.IsDone()) //if its not the first point its gone wrong
		{
			continue;
		}
		else
		{
			TopoDS_Wire wire = wireMaker.Wire();
			innerLoops.Append(wire);   //需要自己求外包框
		}

		//build the face
		//if we have no outer loop defined, find the biggest
		if (outerLoop.IsNull())
		{
			double area = 0;
			int foundIndex = -1;
			int idx = 0;
			for (auto it = innerLoops.cbegin(); it != innerLoops.cend(); ++it)
			{
				idx++;
				double loopArea = ShapeAnalysis::ContourArea(TopoDS::Wire(*it));
				if (loopArea > area)
				{
					outerLoop = TopoDS::Wire(*it);
					area = loopArea;
					foundIndex = idx;
				}
			}
			if (foundIndex > 0)innerLoops.Remove(foundIndex); //remove outer loop from inner loops
		}
		if (outerLoop.IsNull())
		{
			//no bounded face
			continue;
		}

		try
		{
			gp_Dir outerNormal = OCCUtils::Wire::NormalDir(outerLoop); //this can throw an exception if the wire is nonsense (line) and should just be dropped
			TopoDS_Vertex v1, v2;
			TopExp::Vertices(outerLoop, v1, v2);
			gp_Pln thePlane(BRep_Tool::Pnt(v1), outerNormal);
			BRepBuilderAPI_MakeFace faceMaker(thePlane, outerLoop, true);
			if (faceMaker.IsDone())
			{
				if (innerLoops.Size() > 0)
				{
					for (auto it = innerLoops.cbegin(); it != innerLoops.cend(); ++it)
					{
						//ensure it is the correct orientation
						try
						{
							TopoDS_Wire innerWire = TopoDS::Wire(*it);
							gp_Vec innerNormal = OCCUtils::Wire::NormalDir(innerWire);
							if (!outerNormal.IsOpposite(innerNormal, Precision::Angular()))
								innerWire.Reverse();
							faceMaker.Add(innerWire);
						}
						catch (Standard_Failure sf)
						{
							continue;
						}
					}
				}

				builder.Add(shell, faceMaker.Face());
				//allFaces++;
			}
			else
			{
				continue;
			}
		}
		catch (const std::exception&)
		{
			continue;
		}
    }

	//check the shell
	BRepCheck_Shell checker(shell);
	BRepCheck_Status st = checker.Orientation();
	if (st != BRepCheck_Status::BRepCheck_NoError)
	{
		ShapeFix_Shell shellFixer(shell);
		if (shellFixer.Perform())
		{
			shell = shellFixer.Shell();
			checker.Init(shell);
		}
		if (checker.Closed() == BRepCheck_Status::BRepCheck_NoError)
		{
			shell.Closed(true);
			shell.Checked(true);
			return shell;
		}
		else
		{
			ShapeFix_Shape shapeFixer(shell);
			if (shapeFixer.Perform())
				return shapeFixer.Shape();
			else
				return shell;
		}
	}
	else //it is oriented correctly and closed
	{
		shell.Closed(true);
		shell.Checked(true);
		return shell;
	}
	return shell;
}

std::vector<TopoDS_Solid> OCCUtils::Solid::CreateSolidShapeByShell(std::vector<TopoDS_Face> faces)
{
	BRepPrim_Builder builder;
	TopoDS_Shell shell;
	builder.MakeShell(shell);
	for (auto face : faces)
	{
		builder.AddShellFace(shell, face);
	}
	builder.CompleteShell(shell);
	BRepCheck_Shell checker(shell);
	BRepCheck_Status result = checker.Closed();
	BRepBuilderAPI_MakeSolid mBuilder(shell);

	TopoDS_Compound pCompound;
	if (mBuilder.IsDone())
	{
		BRep_Builder b;
		b.MakeCompound(pCompound);
		b.Add(pCompound, mBuilder.Solid());
	}
	else
		return std::vector<TopoDS_Solid>();

	std::vector<TopoDS_Solid> solids;
	TopTools_IndexedMapOfShape map;
	TopExp::MapShapes(pCompound, TopAbs_SOLID, map);
	for (int i = 1; i <= map.Extent(); i++)
	{
		solids.push_back(TopoDS::Solid(map(i)));
	}
	return solids;
}

std::vector<TopoDS_Solid> OCCUtils::Solid::CreateSolidByFaces(std::vector<TopoDS_Face> faces)
{
	auto shape = Solid::CreateSolidShapeByFaces(faces);
	//TopoDS_Shape shape;
	TopoDS_Compound pCompound;
	BRep_Builder builder;
	builder.MakeCompound(pCompound);
	builder.Add(pCompound, shape);

	std::vector<TopoDS_Solid> solids;
	TopTools_IndexedMapOfShape map;
	TopExp::MapShapes(pCompound, TopAbs_SOLID, map);
	for (int i = 1; i <= map.Extent(); i++)
		solids.push_back(TopoDS::Solid(map(i)));
	
	//upgrade any shells that are closed to solids
	std::vector<TopoDS_Solid> resSolids;
	TopExp_Explorer te(shape, TopAbs_SHELL, TopAbs_SOLID);
	while (te.More())
	{
		const TopoDS_Shell& shell = TopoDS::Shell(te.Current());
		if (shell.Closed())
		{
			BRepBuilderAPI_MakeSolid solidMaker(shell);
			if (solidMaker.IsDone())
			{
				resSolids.push_back(solidMaker.Solid());
			}
		}
		te.Next();
	}
	return resSolids;
}

std::vector<TopoDS_Solid> OCCUtils::Solid::CreateSolidWithVoids(std::vector<TopoDS_Face> faces)
{
	auto outerShell = Solid::MakeShell(faces);
	TopoDS_Compound pCompound;
	BRepCheck_Shell checker(outerShell);
	BRepCheck_Status result = checker.Closed();
	if (!result == BRepCheck_NoError) //we have a shell that is not able to be made in to a solid
	{
	}
	BRepBuilderAPI_MakeSolid builder(outerShell);
	/*for each (IIfcClosedShell ^ IIfcVoidShell in brepWithVoids->Voids)			//不考虑洞口
	{
	}*/
	if (builder.IsDone())
	{
		BRep_Builder b;
		b.MakeCompound(pCompound);
		b.Add(pCompound, builder.Solid());
	}
	else
		return std::vector<TopoDS_Solid>();

	std::vector<TopoDS_Solid> solids;
	TopTools_IndexedMapOfShape map;
	TopExp::MapShapes(pCompound, TopAbs_SOLID, map);
	for (int i = 1; i <= map.Extent(); i++)
	{
		solids.push_back(TopoDS::Solid(map(i)));
	}
		
	//BRepBuilderAPI_MakeSolid solidMaker(pCompound);
	//if (solidMaker.IsDone())
	//{
	//	TopoDS_Solid solid = solidMaker.Solid();
	//	try
	//	{
	//		BRepClass3d_SolidClassifier class3d(solid);
	//		class3d.PerformInfinitePoint(Precision::Confusion());
	//		if (class3d.State() == TopAbs_IN)
	//			solid.Reverse();
	//	}
	//	catch (Standard_Failure sf)
	//	{
	//		//String^ err = gcnew String(sf.GetMessageString());	
	//		//XbimGeometryCreator::LogWarning(logger, this, "Could not build a correct solid from the shell: " + err);
	//	}
	//	return gcnew XbimSolid(solid);
	//}
	//std::vector<TopoDS_Solid> solids;
	//IXbimSolid^ solid = dynamic_cast<IXbimSolid^>(shape);
	//if (solid != nullptr && solid->IsValid)
	//{
	//	return solids->Add(solid);
	//}

	//IXbimSolidSet^ solidSet = dynamic_cast<IXbimSolidSet^>(shape);
	//if (solidSet != nullptr) return solids->AddRange(solidSet);
	//IXbimGeometryObjectSet^ geomSet = dynamic_cast<IXbimGeometryObjectSet^>(shape);
	//if (geomSet != nullptr)
	//{
	//	for each (IXbimGeometryObject ^ geom in geomSet)
	//	{

	//		XbimSolid^ nestedSolid = dynamic_cast<XbimSolid^>(geom);
	//		XbimCompound^ nestedCompound = dynamic_cast<XbimCompound^>(geom);
	//		XbimShell^ shell = dynamic_cast<XbimShell^>(geom);
	//		if (nestedSolid != nullptr && !nestedSolid->IsEmpty)
	//			solids->Add(nestedSolid);
	//		else if (nestedCompound != nullptr)
	//		{
	//			nestedCompound->Sew();
	//			for each (IXbimGeometryObject ^ nestedGeom in nestedCompound)
	//			{
	//				XbimSolid^ subSolid = dynamic_cast<XbimSolid^>(nestedGeom);
	//				XbimShell^ subShell = dynamic_cast<XbimShell^>(nestedGeom);
	//				if (subSolid != nullptr && !subSolid->IsEmpty)
	//					solids->Add(subSolid);
	//				else if (subShell != nullptr && subShell->IsValid)
	//					solids->Add(subShell->MakeSolid());
	//			}
	//		}
	//		else if (shell != nullptr && shell->IsValid)
	//		{
	//			XbimSolid^ s = (XbimSolid^)shell->MakeSolid();
	//			solids->Add(s);
	//		}

	//	}

	//	return;
	//}
	//XbimShell^ shell = dynamic_cast<XbimShell^>(shape);
	//if (shell != nullptr && shell->IsValid/* && shell->IsClosed*/)
	//	return solids->Add(shell->MakeSolid());
	return solids;
}

TopoDS_Compound OCCUtils::Solid::MakeCompound(std::vector<TopoDS_Face> faces)
{
	TopoDS_Compound pCompound;
	TopoDS_Shape occOuterShell = OCCUtils::Solid::CreateSolidShapeByFaces(faces);
	if (occOuterShell.IsNull())
	{
		return pCompound;
	}
	BRep_Builder b;	
	b.MakeCompound(pCompound);
	if (occOuterShell.ShapeType() == TopAbs_SHELL && occOuterShell.Closed())
	{
		BRepBuilderAPI_MakeSolid solidmaker;
		auto s = TopoDS::Shell(occOuterShell);
		BRepCheck_Shell checker(s);
		BRepCheck_Status result = checker.Closed();

		solidmaker.Add(TopoDS::Shell(occOuterShell));
		solidmaker.Build();
		if (solidmaker.IsDone())
		{
			TopoDS_Solid s = solidmaker.Solid();
			s.Closed(true);
			s.Checked(true);
			b.Add(pCompound, s);
		}
		pCompound.Closed(true);
		pCompound.Checked(true);
		return pCompound;
	}

	//manifold breps are always solids, so to make sure we have highest form, sometime we get multiple solids
	try
	{

		TopTools_IndexedMapOfShape shellMap;
		TopExp::MapShapes(occOuterShell, TopAbs_SHELL, shellMap);
		for (int ishell = 1; ishell <= shellMap.Extent(); ++ishell)
		{
			// Build solid
			BRepBuilderAPI_MakeSolid solidmaker;
			const TopoDS_Shell& shell = TopoDS::Shell(shellMap(ishell));
			solidmaker.Add(shell);
			solidmaker.Build();
			if (solidmaker.IsDone())
			{
				TopoDS_Solid s = solidmaker.Solid();
				BRepClass3d_SolidClassifier class3d(s);
				class3d.PerformInfinitePoint(Precision::Confusion());
				if (class3d.State() == TopAbs_IN) s.Reverse();
				b.Add(pCompound, s);
			}
		}
	}
	catch (Standard_Failure sf)
	{
		b.Add(pCompound, occOuterShell); //just add what we have
	}
	return pCompound;
}

TopoDS_Shell OCCUtils::Solid::MakeShell(std::vector<TopoDS_Face> faces)
{
	auto pCompound = Solid::MakeCompound(faces);
	if (Solid::Count(&pCompound) == 1) {
		for (TopExp_Explorer expl(pCompound, TopAbs_SOLID); expl.More();)
		{
			TopoDS_Solid pSolid = TopoDS::Solid(expl.Current());
			if (&pSolid != nullptr && !(&pSolid)->IsNull()) {
				TopTools_IndexedMapOfShape map;
				TopExp::MapShapes(pSolid, TopAbs_SHELL, map);
				std::vector<TopoDS_Shell> shells;
				for (int i = 1; i <= map.Extent(); i++)
					shells.push_back(TopoDS::Shell(map(i)));
				if (shells.size() == 1)
				{
					return shells[0];
				}
			}
		}
		for (TopExp_Explorer expl(pCompound, TopAbs_SHELL, TopAbs_SOLID); expl.More();)
		{
			return TopoDS::Shell(expl.Current());
		}
	}
	//make all the faces in to one shell, this may be a topologically illegal object
	BRepPrim_Builder builder;
	TopoDS_Shell shell;
	builder.MakeShell(shell);
	TopTools_IndexedMapOfShape map;
	TopExp::MapShapes(pCompound, TopAbs_FACE, map);
	for (int i = 1; i <= map.Extent(); i++)
	{
		builder.AddShellFace(shell, TopoDS::Face(map(i)));
	}
	builder.CompleteShell(shell);
	return shell;
}

int OCCUtils::Solid::Count(TopoDS_Compound* pCompound)
{
	if (pCompound == nullptr) return 0;
	int count = 0;
	for (TopExp_Explorer expl(*pCompound, TopAbs_SOLID); expl.More(); expl.Next())
		count++;
	for (TopExp_Explorer expl(*pCompound, TopAbs_SHELL, TopAbs_SOLID); expl.More(); expl.Next())
		count++;
	for (TopExp_Explorer expl(*pCompound, TopAbs_FACE, TopAbs_SHELL); expl.More(); expl.Next())
		count++;
	return count;
}