#include "FollowPathNode.h"

#include "../../Bot.h"
#include "../../GameProxy.h"
#include "../../RayCaster.h"

namespace marvin {
namespace behavior {

ExecuteResult FollowPathNode::Execute(ExecuteContext& ctx) {
  auto path = ctx.blackboard.ValueOr(path_name_, std::vector<Vector2f>());
  size_t path_size = path.size();

  if (path.empty()) return ExecuteResult::Failure;

  auto& game = ctx.bot->GetGame();
  Vector2f current = path.front();
  Vector2f from = game.GetPosition();

  // Fix an issue where the pathfinder generates nodes in the center of the tile
  // but the ship can't occupy the exact center. This is only an issue without path smoothing.
  if (!path.empty() && (u16)from.x == (u16)path.at(0).x && (u16)from.y == (u16)path.at(0).y) {
    path.erase(path.begin());

    if (!path.empty()) {
      current = path.front();
    }
  }

  while (path.size() > 1 &&
         CanMoveBetween(game, from, path.at(1))) {
    path.erase(path.begin());
    current = path.front();
  }

  if (path.size() == 1 && path.front().DistanceSq(game.GetPosition()) < 2 * 2) {
    path.clear();
  }

  if (path.size() != path_size) {
    ctx.blackboard.Set(path_name_, path);
  }

  //ctx.bot->GetSteering().Arrive(current, 0.3f);
  ctx.bot->Move(current, 0.0f);

  return ExecuteResult::Success;
}

bool FollowPathNode::CanMoveBetween(GameProxy& game, Vector2f from,
                                    Vector2f to) {
  Vector2f trajectory = to - from;
  Vector2f direction = Normalize(trajectory);
  Vector2f side = Perpendicular(direction);

  float distance = from.Distance(to);
  float radius = game.GetShipSettings().GetRadius();

  CastResult center = RayCast(game.GetMap(), from, direction, distance);
  CastResult side1 =
      RayCast(game.GetMap(), from + side * radius, direction, distance);
  CastResult side2 =
      RayCast(game.GetMap(), from - side * radius, direction, distance);

  return !center.hit && !side1.hit && !side2.hit;
}

}  // namespace behavior
}  // namespace marvin
