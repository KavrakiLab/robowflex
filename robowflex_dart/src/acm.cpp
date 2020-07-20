#include <robowflex_dart/acm.h>
#include <robowflex_dart/structure.h>

using namespace robowflex::darts;

///
/// ACM
///

ACM::ACM(const Structure *structure)
  : structure_(structure), filter_(std::make_shared<dart::collision::BodyNodeCollisionFilter>())
{
}

void ACM::disableCollision(const std::string &a, const std::string &b)
{
    auto key = makeKey(a, b);
    if (acm_.find(key) == acm_.end())
    {
        acm_.emplace(key);
        filter_->addBodyNodePairToBlackList(getBodyNode(key.first), getBodyNode(key.second));
    }
}

void ACM::enableCollision(const std::string &a, const std::string &b)
{
    auto key = makeKey(a, b);
    auto it = acm_.find(key);
    if (it != acm_.end())
    {
        filter_->removeBodyNodePairFromBlackList(getBodyNode(key.first), getBodyNode(key.second));
        acm_.erase(it);
    }
}

std::shared_ptr<dart::collision::BodyNodeCollisionFilter> ACM::getFilter()
{
    return filter_;
}

const std::shared_ptr<dart::collision::BodyNodeCollisionFilter> &ACM::getFilterConst() const
{
    return filter_;
}

const Structure *ACM::getStructure() const
{
    return structure_;
}

std::set<std::pair<std::string, std::string>> &ACM::getDisabledPairs()
{
    return acm_;
}

const std::set<std::pair<std::string, std::string>> &ACM::getDisabledPairsConst() const
{
    return acm_;
}

std::pair<std::string, std::string> ACM::makeKey(const std::string &a, const std::string &b) const
{
    if (a < b)
        return std::make_pair(a, b);

    return std::make_pair(b, a);
}

dart::dynamics::BodyNode *ACM::getBodyNode(const std::string &key)
{
    return structure_->getSkeletonConst()->getBodyNode(key);
}
