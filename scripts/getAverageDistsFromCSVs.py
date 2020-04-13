#!/usr/bin/env python

import numpy as np
import pandas as pd

def contitionDF(df, colnameSufix, droppedCols):
    df.dropna(inplace=True)
    df = df.reset_index()

    if (droppedCols!=None):
        df.drop(droppedCols, inplace=True,axis=1)

    remapDict = dict()
    for col in df.columns:
        remapDict[col] = col + ' (' + colnameSufix + ')'

    df.rename(columns=remapDict, inplace=True)

    return df


# Main function.
if __name__ == '__main__':
    # params
    # dont forget the final slash!!!!
    dataFolder = '/home/pulver/Dropbox/University/PostDoc/MCDM/mcdm_ral_experiments/inb3123/'
    # distances bigger than this will be considered error and removed
    errorDist = 15
    fileNames = ['distance_inb3123_mcdm_2.csv', 'distance_inb3123_mcdm_0.csv', 'distance_inb3123_mcdm_1.csv']


    # just in case you are copypasting ...
    if 'allData' in locals():
        del allData

    for fileName in fileNames:      
        #fileName = fileNames[0]
        fileURI = dataFolder + fileName
        dataIN = pd.read_csv(fileURI,index_col=False)
        
        for i in range(1,11):
            dataIN.loc[dataIN['tag'+str(i)] > errorDist, 'tag'+str(i)] = errorDist


        # dataIN['tag_mean']=dataIN.iloc[:,-10:].mean(axis=1)
        # dataIN['tag_std']=dataIN.iloc[:,-10:].std(axis=1)
        # dataIN.to_csv(fileURI[:-4] + '_with_av.csv')

        # Stack the DataFrames on top of each other                
        if 'allData' in locals():
            allData = pd.concat([allData, dataIN], axis=0)
        else:
            allData = dataIN

        grouping_colls = ['w_info_gain', 'w_travel_distance', 'w_sensing_time', 'w_rfid_gain', 'w_battery_status']
        averages = allData.groupby(grouping_colls).mean() 
        averages2 = contitionDF(averages, 'av', None)

        stdevs = allData.groupby(grouping_colls).std() 
        droppedCols = ['tag1', 'tag2', 'tag3', 'tag4', 'tag5', 'tag6', 'tag7', 'tag8', 'tag9', 'tag10']
       
        stdevs2 = contitionDF(stdevs, 'std',droppedCols)
        stats = pd.concat([averages2, stdevs2], axis=1)

        # Round everything at 2 decimals
        stats = stats.round(2)

        stats['tag_mean']=stats.iloc[:,5:15].mean(axis=1)
        stats['tag_std']=stats.iloc[:,5:15].std(axis=1)
        # stats.to_csv(fileURI[:-4] + '_with_av.csv')

        # Round everything at 2 decimals
        stats = stats.round(2)

        # Merge avg and std in the same column
        stats["tagAccuracy"] = stats["tag_mean"].astype(str) + "(" + stats["tag_std"].astype(str) + ")"

        # Remove useless column (without reassignment)
        column_to_remove = ['tag1 (av)',
                            'tag2 (av)',
                            'tag3 (av)',
                            'tag4 (av)',
                            'tag5 (av)',
                            'tag6 (av)',
                            'tag7 (av)',
                            'tag8 (av)',
                            'tag9 (av)',
                            'tag10 (av)',
                            'w_info_gain (std)',
                            'w_travel_distance (std)',
                            'w_sensing_time (std)',
                            'w_rfid_gain (std)',
                            'w_battery_status (std)',
                            'tag_mean',
                            'tag_std']
        for column in column_to_remove:
            stats.drop(column, axis=1, inplace=True)


        stats.to_csv('/home/pulver/Desktop/distance.csv', index=False)

        stats_transposed = stats.T
        stats_transposed.to_csv('/home/pulver/Desktop/distance_transposed.csv', header=False)#, index=False)