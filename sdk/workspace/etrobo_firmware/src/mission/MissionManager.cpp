#include "mission/MissionManager.h"

namespace etrobo {
namespace mission {

void MissionManager::initialize()
{
  lineController_.configureDefault();
}

int MissionManager::configureLine(const LineConfig* config)
{
  return lineController_.configure(config);
}

int MissionManager::start()
{
  return lineController_.start();
}

void MissionManager::stop()
{
  lineController_.stop();
}

int MissionManager::applyQr(const QrCorrection* correction)
{
  return lineController_.applyQr(correction);
}

void MissionManager::getStatus(LineStatus& status) const
{
  lineController_.getStatus(status);
}

void MissionManager::runControlCycle()
{
  lineController_.runCycle();
}

}  // namespace mission
}  // namespace etrobo
