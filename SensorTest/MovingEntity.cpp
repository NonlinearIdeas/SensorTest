/********************************************************************
 * File   : MovingEntity.cpp
 * Project: MissileDemo
 *
 ********************************************************************
 * Created on 10/20/13 By Nonlinear Ideas Inc.
 * Copyright (c) 2013 Nonlinear Ideas Inc. All rights reserved.
 ********************************************************************
 * This software is provided 'as-is', without any express or implied
 * warranty.  In no event will the authors be held liable for any
 * damages arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any
 * purpose, including commercial applications, and to alter it and
 * redistribute it freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must
 *    not claim that you wrote the original software. If you use this
 *    software in a product, an acknowledgment in the product
 *    documentation would be appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and
 *    must not be misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source
 *    distribution.
 */


#include "MovingEntity.h"
#include "Notifier.h"
#include "GraphSensorManager.h"


static void DrawNodeList(const list<const GraphNode*>& nodes, ccColor4F color)
{
   //   CCLOG("Drawing Node List (%ld nodes)",nodes.size());
   for(list<const GraphNode*>::const_iterator iter = nodes.begin();
       iter != nodes.end();
       ++iter)
   {
      NavGraphNode* gNode = (NavGraphNode*)*iter;
      LINE_METERS_DATA_T lmd;
      lmd.start = gNode->GetPos();
      lmd.end = lmd.start;
      lmd.color = color;
      lmd.markerRadius = 4.0f;
      Notifier::Instance().Notify<LINE_METERS_DATA_T>(NE_DEBUG_LINE_DRAW_ADD_LINE, lmd);
   }
}

static void DrawNodeList(const list<const GraphNode*>& nodes)
{
   DrawNodeList(nodes, ccc4f(0.99f, 0.25f, 0.25f, 1.0f));
}

static void DrawPathList(const list<Vec2> points)
{
   
   if(points.size() > 0)
   {
      LINE_METERS_DATA_T lmd;
      Vec2 lastPoint = *points.begin();
      for(list <Vec2>::const_iterator iter = ++(points.begin());
          iter != points.end();
          ++iter)
      {
         lmd.start = lastPoint;
         lmd.end = *iter;
         lmd.color = ccc4f(0.99f, 0.45f, 0.25f, 0.75f);
         lmd.markerRadius = 2.0f;
         Notifier::Instance().Notify<LINE_METERS_DATA_T>(NE_DEBUG_LINE_DRAW_ADD_LINE, lmd);
         lastPoint = lmd.end;
      }
   }
}

static void DrawEdgeList(const list<const GraphEdge*>& edges)
{
   Notifier& no = Notifier::Instance();
   CCLOG("Drawing Path List (%ld edges)",edges.size());
   for(list<const GraphEdge*>::const_iterator iter = edges.begin();
       iter != edges.end();
       ++iter)
   {
      NavGraphEdge* gEdge = (NavGraphEdge*)*iter;
      LINE_METERS_DATA_T lmd;
      lmd.start = gEdge->GetSrcPos();
      lmd.end = gEdge->GetDesPos();
      no.Notify<LINE_METERS_DATA_T>(NE_DEBUG_LINE_DRAW_ADD_LINE, lmd);
   }
}


void MovingEntity::StopBody()
{
   GetBody()->SetLinearDamping(4.0);
   GetBody()->SetAngularDamping(2.5);
}


void MovingEntity::SetupTurnController()
{
   GetBody()->SetAngularDamping(0);
   PIDController& turnController = GetTurnController();
   
   turnController.ResetHistory();
   _turnController.ResetHistory();
   _turnController.SetKDerivative(5.0);
   _turnController.SetKProportional(1.0);
   _turnController.SetKIntegral(0.05);
   _turnController.SetKPlant(1.0);
}

void MovingEntity::CreateBody(b2World& world, const b2Vec2& position, float32 angleRads)
{
   // Create the body.
   b2BodyDef bodyDef;
   bodyDef.position = position;
   bodyDef.type = b2_dynamicBody;
   Body* body = world.CreateBody(&bodyDef);
   assert(body != NULL);
   // Store it in the base.
   SetBody(body);
   
   // Now attach fixtures to the body.
   FixtureDef fixtureDef;
   PolygonShape polyShape;
   vector<Vec2> vertices;
   
   const float32 VERT_SCALE = GetSizeMeters();
   fixtureDef.shape = &polyShape;
   fixtureDef.density = 1.0;
   fixtureDef.friction = 1.0;
   fixtureDef.isSensor = false;
   
   // Nose
   vertices.clear();
   vertices.push_back(Vec2(4*VERT_SCALE,2*VERT_SCALE));
   vertices.push_back(Vec2(4*VERT_SCALE,-2*VERT_SCALE));
   vertices.push_back(Vec2(8*VERT_SCALE,-0.5*VERT_SCALE));
   vertices.push_back(Vec2(8*VERT_SCALE,0.5*VERT_SCALE));
   polyShape.Set(&vertices[0],vertices.size());
   body->CreateFixture(&fixtureDef);
   body->SetLinearDamping(0.25);
   body->SetAngularDamping(0.25);
   
   // Main body
   vertices.clear();
   vertices.push_back(Vec2(-4*VERT_SCALE,2*VERT_SCALE));
   vertices.push_back(Vec2(-4*VERT_SCALE,-2*VERT_SCALE));
   vertices.push_back(Vec2(4*VERT_SCALE,-2*VERT_SCALE));
   vertices.push_back(Vec2(4*VERT_SCALE,2*VERT_SCALE));
   polyShape.Set(&vertices[0],vertices.size());
   body->CreateFixture(&fixtureDef);
   
   // Setup Parameters
   SetMaxAngularAcceleration(20*M_PI);
   SetMaxLinearAcceleration(10);
   SetMaxSpeed(5);
   SetMinSeekDistance(0.5);
}

bool MovingEntity::IsNearTarget()
{
   Vec2 toTarget = GetTargetPos() - GetBody()->GetPosition();
   
   if(toTarget.LengthSquared() < GetMinSeekDistance()*GetMinSeekDistance())
   {
      return true;
   }
   return false;
}

bool MovingEntity::IsNearTarget(const Vec2& target,float32 distanceMin)
{
   Vec2 toTarget = target - GetBody()->GetPosition();
   
   if(toTarget.LengthSquared() < distanceMin*distanceMin)
   {
      return true;
   }
   return false;
}

bool MovingEntity::Notify(NOTIFIED_EVENT_TYPE_T eventType, const bool& value)
{
   bool result = true;
   switch (eventType)
   {
      case NE_DEBUG_NEXT_ALGORITHM:
         _navAlg++;
         if(_navAlg >= NA_MAX)
         {
            _navAlg = NA_FIRST;
         }
         break;
      default:
         result = false;
         break;
   }
   return result;
}


void MovingEntity::ApplyTurnTorque()
{
   Vec2 toTarget = GetTargetPos() - GetBody()->GetPosition();
   
   float32 angleBodyRads = MathUtilities::AdjustAngle(GetBody()->GetAngle());
   float32 angleTargetRads = MathUtilities::AdjustAngle(atan2f(toTarget.y, toTarget.x));
   float32 angleError = MathUtilities::AdjustAngle(angleBodyRads - angleTargetRads);
   _turnController.AddSample(angleError);
   
   // Negative Feedback
   float32 angAcc = -_turnController.GetLastOutput();
   
   // This is as much turn acceleration as this
   // "motor" can generate.
   if(angAcc > GetMaxAngularAcceleration())
      angAcc = GetMaxAngularAcceleration();
   if(angAcc < -GetMaxAngularAcceleration())
      angAcc = -GetMaxAngularAcceleration();
   
   float32 torque = angAcc * GetBody()->GetInertia();
   GetBody()->ApplyTorque(torque);
}

void MovingEntity::ApplyThrust()
{
   // Get the distance to the target.
   Vec2 toTarget = GetTargetPos() - GetBody()->GetWorldCenter();
   toTarget.Normalize();
   Vec2 desiredVel = GetMaxSpeed()*toTarget;
   Vec2 currentVel = GetBody()->GetLinearVelocity();
   Vec2 thrust = GetMaxLinearAcceleration()*(desiredVel - currentVel);
   GetBody()->ApplyForceToCenter(thrust);
}

void MovingEntity::PrepareForMotion()
{
   GetBody()->SetLinearDamping(0.0f);
   GetBody()->SetAngularDamping(0.0f);
}

void MovingEntity::EnterSeek()
{
   PrepareForMotion();
   GetTurnController().ResetHistory();
}

void MovingEntity::ExecuteSeek()
{
   if(IsNearTarget())
   {
      StopBody();
      ChangeState(ST_IDLE);
   }
   else
   {
      ApplyTurnTorque();
      ApplyThrust();
   }
}


void MovingEntity::EnterIdle()
{
   StopBody();
}

void MovingEntity::ExecuteIdle()
{
}

void MovingEntity::EnterTurnTowards()
{
   PrepareForMotion();
   GetTurnController().ResetHistory();
}

void MovingEntity::ExecuteTurnTowards()
{
   ApplyTurnTorque();
}


void MovingEntity::EnterFollowPath()
{
   // If there are any points to follow,
   // then pop the first as the target
   // and follow it.  Otherwise, go idle.
   list<Vec2>& path = GetPath();
   if(path.size() > 0)
   {
      PrepareForMotion();
      GetTurnController().ResetHistory();
      GetTargetPos() = *(path.begin());
      path.erase(path.begin());
   }
   else
   {
      ChangeState(ST_IDLE);
   }
}

void MovingEntity::ExecuteFollowPath()
{
   list<Vec2>& path = GetPath();
   bool isNearTarget = IsNearTarget();
   if(path.size() == 0)
   {  // Out of points in the list
      if(isNearTarget)
      {
         ChangeState(ST_IDLE);
      }
      else
      {  // Just keep pushing towards target.
         ApplyThrust();
         ApplyTurnTorque();
      }
   }
   else
   {  // Still more to go
      if(isNearTarget)
      {
         GetTargetPos() = *(path.begin());
         path.erase(path.begin());
      }
      ApplyThrust();
      ApplyTurnTorque();
   }
}

bool MovingEntity::FindPath(const Vec2& startPos, const Vec2& endPos, list<Vec2>& path)
{
   GraphSensorManager& gsm = GraphSensorManager::Instance();
   const GridCalculator& gridCalculator = gsm.GetGridCalculator();
   const Graph& sensorGraph = gsm.GetSensorGraph();
   
   int32 startIdx = gridCalculator.CalcIndex(GetBody()->GetWorldCenter());
   int32 targetIdx = gridCalculator.CalcIndex(_navigatePos);
   if(startIdx == targetIdx)
   {
      CCLOG("Start and end at same position...(%d)",startIdx);
   }
   else
   {
      //      CCLOG("Searching for path from %d --> %d",startIdx,targetIdx);
   }
   GraphSearchAlgorithm* gsa = NULL;
   AStarHeuristic* ashe = NULL;
   
   switch(_navAlg)
   {
      case NA_BFS:
         gsa = new GraphSearchBFS(sensorGraph,startIdx,targetIdx);
         break;
      case NA_DIJ:
         gsa = new GraphSearchDijkstra(sensorGraph,startIdx,targetIdx);
         break;
      case NA_AST_DIST:
         ashe = new AStarHeuristic_Distance();
         gsa = new GraphSearchAStar(sensorGraph,startIdx,targetIdx,ashe);
         break;
      case NA_AST_DISTSQ:
         ashe = new AStarHeuristic_DistanceSquared();
         gsa = new GraphSearchAStar(sensorGraph,startIdx,targetIdx,ashe);
         break;
      case NA_AST_MANHATTAN:
         ashe = new AStarHeuristic_DistanceManhattan();
         gsa = new GraphSearchAStar(sensorGraph,startIdx,targetIdx,ashe);
         break;
      default:
         assert(false);
         break;
   }
   _searchStat.Start();
   GraphSearchAlgorithm::SEARCH_STATE_T sstate = gsa->SearchGraph();
   _searchStat.Stop(gsa->GetFloodNodes().size());
   CCLOG("Average Search Time %.2f ms, Average Nodes = %.1f (%.2f s total, %d samples, %.1f Nodes)",
         _searchStat.GetAverageSeconds()*1000,
         _searchStat.GetAverageValue(),
         _searchStat.GetTotalSeconds(),
         _searchStat.GetSamples(),
         _searchStat.GetValue()
         );
   if(sstate == GraphSearchAlgorithm::SS_FOUND)
   {
      list<const GraphNode*> pathNodes;
      
      pathNodes = gsa->GetPathNodes();
      
      Notifier::Instance().Notify(NE_RESET_DRAW_CYCLE, true);
      DrawNodeList(gsa->GetFloodNodes());
      path.clear();
      for(list<const GraphNode*>::const_iterator iter = pathNodes.begin();
          iter != pathNodes.end();
          ++iter)
      {
         const NavGraphNode* gNode = (NavGraphNode*)*iter;
         path.push_back(gNode->GetPos());
      }
      DrawPathList(path);
   }
   else
   {
      //     Notifier::Instance().Notify(NE_RESET_DRAW_CYCLE, true);
      //   DrawNodeList(search.GetFloodNodes());
      
      switch(sstate)
      {
         case GraphSearchAlgorithm::SS_NOT_FOUND:
            CCLOG("No Path Found!!!");
            break;
         case GraphSearchAlgorithm::SS_NOT_STARTED:
            assert(false);
            break;
         case GraphSearchAlgorithm::SS_STILL_WORKING:
            assert(false);
            break;
         default:
            assert(false);
            break;
      }
   }
   // Don't leak memory!!!
   if(gsa != NULL)
      delete gsa;
   if(ashe != NULL)
      delete ashe;
   return sstate == GraphSearchAlgorithm::SS_FOUND;
}


void MovingEntity::EnterNavigateToPoint()
{
   if(FindPath(GetBody()->GetWorldCenter(), _navigatePos, _path))
   {
      if(_path.size() > 0)
      {  // Erase the first point...it is usually  junk.
         _path.erase(_path.begin());
      }
      _path.push_back(_navigatePos);
      PrepareForMotion();
      GetTurnController().ResetHistory();
      GetTargetPos() = *(_path.begin());
      _path.erase(_path.begin());
      ResetStateTickTimer(2*TICKS_PER_SECOND);
   }
   else
   {
      ChangeState(ST_IDLE);
   }
}

void MovingEntity::ExecuteNavigateToPoint()
{
   list<Vec2>& path = GetPath();
   bool isNearTarget = IsNearTarget();
   bool isNearNavTarget = IsNearTarget(_navigatePos,5.0);
   const GridCalculator& gridCalc = GraphSensorManager::Instance().GetGridCalculator();
   int32 navigateIdx = gridCalc.CalcIndex(_navigatePos);

   /* If the state tick timer expires, it means we
    * have spent too long trying to reach the next waypoint
    * and we need to replan on how to get to it.
    */
   if(IsStateTickTimerExpired())
   {
      CCLOG("Tick Timer Expired!!!");
      ChangeState(ST_NAVIGATE_TO_POINT);
   }
   /* If we are close to the navigation target point,
    * just seek to it.
    */
   if(isNearNavTarget)
   {  // Must be really close...just seek to it.
      CommandSeek(_navigatePos);
   }
   else
   {  // If we are near the target and there are more points
      // on the list, pop the next point and navigate to it.
      if(isNearTarget && path.size() > 0)
      {  // Still more points on the list.
         GetTargetPos() = *(path.begin());
         path.pop_front();
         ResetStateTickTimer(2*TICKS_PER_SECOND);
         /* If we can't get past the current nodes, replan.
          */
         Vec2 currentPoint = GetTargetPos();
         int32 currentIdx = gridCalc.CalcIndex(currentPoint);
         if(currentIdx != navigateIdx && !IsNodePassable(currentIdx)  )
         {
            ChangeState(ST_NAVIGATE_TO_POINT);
         }
      }
      ApplyThrust();
      ApplyTurnTorque();
   }
}

bool IsPathPassable(const list<Vec2>&path, int32 lookAhead = 3)
{
   return false;
}


bool MovingEntity::IsNodePassable(int32 nodeIdx)
{
   GraphSensorManager& gsm = GraphSensorManager::Instance();
   list<const GraphNode*> nodesChecked;
   bool result = true;
   const GraphNode* node = gsm.GetSensorGraph().GetNode(nodeIdx);
   nodesChecked.push_back(node);
   if(node->IsFlagClear(HasFlags::HF_IS_CONNECTED))
   {  // The cell is blocked...we need to replan.
      result = false;
   }
   else
   {
      // Just check the surrounding nodes...if any of them are blocked, the
      // node is considered blocked.  This is much simpler than
      // trying to user the direction of travel.
      const GraphEdges& edges = gsm.GetSensorGraph().GetEdges(nodeIdx);
      for(int32 idx = 0; idx < edges.size() && result; ++idx)
      {
         node = gsm.GetSensorGraph().GetNode(edges[idx]->GetDes());
         nodesChecked.push_back(node);
         if(node->IsFlagClear(HasFlags::HF_IS_CONNECTED))
         {
            result = false;
         }
      }
   }
   DrawNodeList(nodesChecked,ccc4f(0.99, 0.99, 0.99, 1.0));
   return result;
}


void MovingEntity::ExecuteState(STATE_T state)
{
   UpdateStateTickTimer();
   switch(state)
   {
      case ST_IDLE:
         ExecuteIdle();
         break;
      case ST_TURN_TOWARDS:
         ExecuteTurnTowards();
         break;
      case ST_SEEK:
         ExecuteSeek();
         break;
      case ST_FOLLOW_PATH:
         ExecuteFollowPath();
         break;
      case ST_NAVIGATE_TO_POINT:
         ExecuteNavigateToPoint();
         break;
      default:
         assert(false);
   }
}

void MovingEntity::EnterState(STATE_T state)
{
   ResetStateTickTimer(0);
   switch(state)
   {
      case ST_IDLE:
         EnterIdle();
         break;
      case ST_TURN_TOWARDS:
         EnterTurnTowards();
         break;
      case ST_SEEK:
         EnterSeek();
         break;
      case ST_FOLLOW_PATH:
         EnterFollowPath();
         break;
      case ST_NAVIGATE_TO_POINT:
         EnterNavigateToPoint();
         break;
      default:
         assert(false);
   }
}

void MovingEntity::ChangeState(STATE_T state)
{
   EnterState(state);
   _state = state;
}

MovingEntity::MovingEntity() :
Entity(HF_CAN_MOVE,2),
_state(ST_IDLE),
_navAlg(NA_FIRST)
{
}

MovingEntity::~MovingEntity()
{
   
}

bool MovingEntity::Create(b2World& world,const Vec2& position, float32 angleRads)
{
   CreateBody(world,position,angleRads);
   SetupTurnController();
   Notifier::Instance().Attach(this, NE_DEBUG_NEXT_ALGORITHM);
   return true;
}

// Commands - Use thse to change the state
// of the missile.
void MovingEntity::CommandFollowPath(const list<Vec2>& path)
{
   GetPath() = path;
   ChangeState(ST_FOLLOW_PATH);
}


void MovingEntity::CommandTurnTowards(const Vec2& position)
{
   GetTargetPos() = position;
   ChangeState(ST_TURN_TOWARDS);
}

void MovingEntity::CommandSeek(const Vec2& position)
{
   GetTargetPos() = position;
   ChangeState(ST_SEEK);
}

void MovingEntity::CommandIdle()
{
   ChangeState(ST_IDLE);
}

void MovingEntity::Update()
{
   ExecuteState(_state);
   UpdateDisplay();
}

void MovingEntity::CommandNavigateToPoint(const Vec2& position)
{
   _navigatePos = position;
   ChangeState(ST_NAVIGATE_TO_POINT);
}
