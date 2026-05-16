/*
 * AbstractMST.h
 *
 *  Created on: 22 Mar 2025
 *      Author: tam
 */

#ifndef DISTANCEMAP_SRC_ABSTRACTMST_H_
#define DISTANCEMAP_SRC_ABSTRACTMST_H_

/**
 * @file AbstractMST.hpp
 * @brief Builds a Minimum Spanning Tree across AbstractNodes.
 * @details The base graph (baseNodes + baseEdges) is the thin-skeleton output
 * of GridToGraph. AbstractNodes are clusters of nearby baseNodes. This pass
 * computes the cheapest set of paths through the base graph that connects
 * every AbstractNode, returned as AbstractEdges (each with the underlying
 * base-graph path).
 */

#include "GridTypes.hpp"

namespace DistanceMap {

/**
 * @class AbstractMST
 * @brief Minimum Spanning Tree over AbstractNodes routed through the base graph.
 */
class AbstractMST {
public:
	AbstractMST();
	virtual ~AbstractMST();

	/**
	 * @brief Compute MST AbstractEdges connecting every AbstractNode.
	 * @param graph         Adjacency over baseNodes (excludes dead ends).
	 * @param edges         All base edges (with paths between baseNodes).
	 * @param baseNodes     All baseNode positions.
	 * @param abstractNodes Clusters of baseNodes to connect.
	 * @return AbstractEdges forming the MST; each carries the full base-graph
	 *         path between its two abstract endpoints.
	 */
	static std::vector<GridType::AbstractEdge> generateMSTAbstractEdges(const GridType::BaseGraph& graph,
		const std::vector<GridType::Edge>& edges,
		const std::vector<GridType::Point>& baseNodes,
		const std::vector<GridType::AbstractNode>& abstractNodes);
};
} /* namespace DistanceMap */

#endif /* DISTANCEMAP_SRC_ABSTRACTMST_H_ */
