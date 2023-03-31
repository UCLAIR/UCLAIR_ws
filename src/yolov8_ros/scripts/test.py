for i, row in bottles_df.iterrows():
    # Find the matching row(s) in dataf
    matches_df = self.dataf.loc[
        (self.dataf['character'] == row['character']) &
        (self.dataf['color_shape'] == row['color_shape']) &
        (self.dataf['color_char'] == row['color_char'])
    ]
    
    # If there are no matches, continue to the next row
    if matches_df.empty:
        continue
    
    # Otherwise, find the row with the minimum Distance value
    min_dist_row = matches_df.loc[matches_df['Distance'].idxmin()]
    
    # Add the matching row to the new dataframe
    matching_rows = matching_rows.append(min_dist_row, ignore_index=True)
