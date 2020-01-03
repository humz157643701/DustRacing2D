// This file is part of Dust Racing 2D.
// Copyright (C) 2015 Jussi Lind <jussi.lind@iki.fi>
//
// Dust Racing 2D is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// Dust Racing 2D is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with Dust Racing 2D. If not, see <http://www.gnu.org/licenses/>.

#include "route.hpp"
#include "targetnodebase.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>

#include "simple_logger.hpp"

Route::Route()
{
}

void Route::clear()
{
    m_route.clear();
}

bool Route::push(TargetNodeBasePtr node)
{
    node->setIndex(m_route.size());
    m_route.push_back(node);
    return isClosed();
}

bool Route::isClosed() const
{
    if (m_route.size() > 1)
    {
        const auto dx = std::abs(m_route[0]->location().x() - m_route.back()->location().x());
        const auto dy = std::abs(m_route[0]->location().y() - m_route.back()->location().y());
        const auto closingThreshold = 32;
        return dx < closingThreshold && dy < closingThreshold;
    }

    return false;
}

size_t Route::numNodes() const
{
    return static_cast<size_t>(m_route.size());
}

TargetNodeBasePtr Route::get(size_t index) const
{
    return m_route.at(index % m_route.size());
}

TargetNodeBasePtr Route::get(size_t index, double distance) const
{
    const auto sourceNode = get(index);
    auto bestNode = sourceNode;
    double cumDistance = 0;
    size_t count = 0;
    while (count < m_route.size())
    {
        const auto targetNode = get(index + count);
        cumDistance += distanceBetweenNodes(sourceNode, targetNode);
        bestNode = targetNode;
        if (cumDistance > distance)
        {
            return bestNode;
        }

        count++;
    }
    return bestNode;
}

void Route::getAll(RouteVector & routeVector) const
{
    routeVector = m_route;
}

void Route::buildFromVector(RouteVector & routeVector)
{
    juzzlin::L().debug() << "Building route from vector of " << routeVector.size() << " nodes";

    clear();

    std::sort(routeVector.begin(), routeVector.end(),
              [](const auto lhs, const auto rhs) {
                  return lhs->index() < rhs->index();
              });

    for (auto && node : routeVector)
    {
        push(node);
    }
}

double Route::distanceBetweenNodes(TargetNodeBasePtr node0, TargetNodeBasePtr node1) const
{
    const auto dx = node0->location().x() - node1->location().x();
    const auto dy = node0->location().y() - node1->location().y();

    return std::sqrt(dx * dx + dy * dy);
}

double Route::geometricLength() const
{
    double result = 0;

    if (m_route.size() > 1)
    {
        for (size_t i = 0; i < m_route.size() - 1; i++)
        {
            result += distanceBetweenNodes(m_route[i], m_route[i + 1]);
        }

        result += distanceBetweenNodes(m_route.back(), m_route.front());
    }

    return result;
}

QPointF Route::avgLocation(TargetNodeBasePtr node0, TargetNodeBasePtr node1) const
{
    return (node0->location() + node1->location()) * 0.5;
}

QSizeF Route::maxSize(TargetNodeBasePtr node0, TargetNodeBasePtr node1) const
{
    return { std::max(node0->size().width(), node1->size().width()), std::max(node0->size().height(), node1->size().height()) };
}

size_t Route::firstViolatingNode(RouteVector & routeVector, double maxDistance) const
{
    size_t index = routeVector.size();
    for (size_t i = 0; i + 1 < routeVector.size(); i++)
    {
        if (distanceBetweenNodes(routeVector[i], routeVector[i + 1]) > maxDistance)
        {
            return i;
        }
    }

    if (distanceBetweenNodes(routeVector.back(), routeVector.front()) > maxDistance)
    {
        return routeVector.size() - 1;
    }

    return index;
}

void Route::insertNode(RouteVector & routeVector, size_t index)
{
    auto newNode = std::make_shared<TargetNodeBase>();
    newNode->setSynthetic(true);

    if (index == routeVector.size() - 1)
    {
        newNode->setLocation(avgLocation(routeVector.at(index), routeVector[0]));
        newNode->setSize(maxSize(routeVector.at(index), routeVector[0]));
        routeVector.push_back(newNode);
    }
    else
    {
        newNode->setLocation(avgLocation(routeVector.at(index), routeVector.at(index + 1)));
        newNode->setSize(maxSize(routeVector.at(index), routeVector.at(index + 1)));
        routeVector.insert(routeVector.begin() + static_cast<int>(index) + 1, newNode);
    }

    relinkNodes(routeVector);
}

void Route::relinkNodes(RouteVector & routeVector)
{
    for (size_t i = 0; i + 1 < routeVector.size(); i++)
    {
        routeVector[i]->setIndex(i);
        routeVector[i]->setNext(routeVector[i + 1]);

        if (i > 0)
        {
            routeVector[i]->setPrev(routeVector[i - 1]);
        }
    }

    routeVector.front()->setPrev(routeVector.back());
    routeVector.back()->setNext(routeVector.front());
}

void Route::makeDense(double maxDistance)
{
    juzzlin::L().debug() << "Making the route dense with max distance " << maxDistance;

    size_t iterations = 1;
    size_t index = firstViolatingNode(m_route, maxDistance);
    const size_t oldNodeCount = m_route.size();
    while (index < m_route.size())
    {
        juzzlin::L().debug() << "Violating index: " << index;
        insertNode(m_route, index);
        index = firstViolatingNode(m_route, maxDistance);
        iterations++;
    }

    juzzlin::L().debug() << "Iteration(s): " << iterations;
    juzzlin::L().debug() << "Old node count: " << oldNodeCount;
    juzzlin::L().debug() << "New node count: " << m_route.size();
}

Route::RouteVector::iterator Route::begin()
{
    return m_route.begin();
}

Route::RouteVector::iterator Route::end()
{
    return m_route.end();
}

Route::RouteVector::const_iterator Route::cbegin() const
{
    return m_route.cbegin();
}

Route::RouteVector::const_iterator Route::cend() const
{
    return m_route.cend();
}
