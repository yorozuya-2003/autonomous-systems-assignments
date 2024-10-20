from constants import items
from helpers import generate_bidder_mapping, items_to_bidders_mapping

from itertools import combinations, product


def vcg(items_to_bidders, total_items, bidders):
    all_combinations = [
        item for comb in range(1, total_items + 1)
        for item in combinations(items_to_bidders.keys(), comb)
    ]

    VCG_results = {}

    for auction_item in all_combinations:
        all_items = set()
        
        if all(item not in all_items and not all_items.add(item) for group in auction_item for item in group.split("_and_")):
            args = [items_to_bidders[item_group] for item_group in auction_item]
            max_value = 0
            auction_item_result = None

            for comb in product(*args):
                bidders_used = set()
                curr_sum = 0
                item_group_result = []

                for bidder, bid in comb:
                    if bidder in bidders_used:
                        break
                    curr_sum += bidders[bidder][bid]["value"]
                    item_group_result.append((bidder, bid))
                    bidders_used.add(bidder)

                if len(item_group_result) == len(bidders_used) and curr_sum > max_value:
                    max_value = curr_sum
                    auction_item_result = item_group_result
            
            if auction_item_result:
                VCG_results[auction_item] = auction_item_result

    best_result, max_value = None, 0
    
    for auction_item, item_results in VCG_results.items():
        total_value = sum(bidders[bidder][bid]["value"] for bidder, bid in item_results)
        
        if total_value > max_value or (total_value == max_value and len(auction_item) < len(best_result[0] if best_result else [])):
            max_value = total_value
            best_result = (auction_item, item_results)

    return best_result, max_value


def calculate_social_cost(result, bidders, items):
    social_cost = {}

    for bidder, bid in result[1]:
        temp_bidders = {k: v for k, v in bidders.items() if k != bidder}

        temp_items_to_bidders = items_to_bidders_mapping(len(items), temp_bidders, items)
        value_a = vcg(temp_items_to_bidders, len(items), bidders)[1]

        temp_items = [item for item in items if item not in bidders[bidder][bid]["items"]]
        temp_items_to_bidders = items_to_bidders_mapping(len(temp_items), temp_bidders, temp_items)
        value_b = vcg(temp_items_to_bidders, len(temp_items), bidders)[1]

        social_cost[bidder] = social_cost.get(bidder, 0) + (value_a - value_b)

    return {bidder: social_cost.get(bidder, 0) for bidder in bidders}


class VCG:
    def __init__(self, items, bidders):
        items_to_bidders = items_to_bidders_mapping(len(items), bidders, items)
        result, _ = vcg(items_to_bidders, len(items), bidders)
        
        self.social_cost = calculate_social_cost(result, bidders, items)
        self.auctioner_revenue = sum(self.social_cost.values())
        
        self.result = [(bidder, bid) for bidder, bid in result[1]]

    def __str__(self):
        return (f"vcg auction result: {self.result}\n"
                f"total auctioner revenue: {self.auctioner_revenue}\n"
                f"total cost incurred by bidders: {self.social_cost}")


if __name__ == '__main__':
    csv_file = 'simple_bids.csv'
    bidders = generate_bidder_mapping(csv_file)

    print(VCG(items, bidders))
