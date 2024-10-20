from constants import items

import pandas as pd
from itertools import combinations


def item_list(symbol):
    if symbol == 't': return ['top_banner']
    if symbol == 's': return ['side_banner']
    return ['top_banner', 'side_banner']


def generate_bid_mapping(csv_file_path):
    df = pd.read_csv(csv_file_path)

    mapping = {
        'top_banner': [],
        'side_banner': [],
        'top_banner_and_side_banner': []
    }

    for index, row in df.iterrows():
        item_key = None

        if row['item'] == 't':
            item_key = 'top_banner'
        elif row['item'] == 's':
            item_key = 'side_banner'
        elif row['item'] == 'b':
            item_key = 'top_banner_and_side_banner'

        if item_key is not None:
            mapping[item_key].append((f'bidder{row["bidder_id"]}', f'bid{row["bid"]}'))

    return mapping


def generate_bidder_mapping(csv_file_path):
    df = pd.read_csv(csv_file_path)
    
    mapping = {}

    for _, row in df.iterrows():
        bidder_key = f'bidder{row["bidder_id"]}'
        bid_key = f'bid{row["bid"]}'
        
        if bidder_key not in mapping:
            mapping[bidder_key] = {}
        
        mapping[bidder_key][bid_key] = {
            'value': row['bid_value'],
            'items': item_list(row['item'])
        }

    return mapping


def items_to_bidders_mapping(n, bidder_mapping, items):
    items_to_bidders = {}

    for comb in range(1, n + 1):
        for item_comb in combinations(items, comb):
            item_key = "_and_".join(sorted(item_comb, reverse=True))
            items_to_bidders[item_key] = []

    for bidder, bids in bidder_mapping.items():
        for bid_key, bid_info in bids.items():
            bid_items_key = "_and_".join(sorted(bid_info["items"], reverse=True))
            if bid_items_key in items_to_bidders:
                items_to_bidders[bid_items_key].append((bidder, bid_key))

    return items_to_bidders


if __name__ == '__main__':
    bid_mapping = generate_bid_mapping(csv_file_path ='simple_bids.csv')
    bidder_mapping = generate_bidder_mapping(csv_file_path='simple_bids.csv')

    print(bid_mapping)
    print(bidder_mapping)
    print(items_to_bidders_mapping(len(items), bidder_mapping, items))
