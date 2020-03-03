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
    dataFolder = '/home/pulver/Dropbox/University/PostDoc/MCDM/mcdm_ral_experiments/'
    # experimentFolders = ['inb3123', 'inbatrium', 'inbeng', 'ncfm', 'orebro' ]
    experimentFolders = ['inb3123', 'ncfm', 'orebro' ]
    # Experiment batches 
    numBatches = 3 
    errorDist = 15
    finalData = None

    # some debug output
    print('Using data folder: ['+ dataFolder + ']')
    print('Experiments: '+ '\n\t- ' + '\n\t- '.join(experimentFolders))
        
    for experiment in experimentFolders: 
            # just in case you are copypasting ...
        if 'allData' in locals():
            del allData
            del distanceAllData

        print('- Processing: '+ experiment)
        # experiment = experimentFolders[0] 
        experimentFolder = dataFolder + experiment + '/'
        for i in range(0,numBatches):
            print('- Batch: '+ str(i))
            # i = 0
            # FIXME: careful with number padding (01 or just 1?)
            fileName = 'result_' + experiment + '_mcdm_r'+ str(i) + '.csv'
            distanceFileName = 'distance_' + experiment + '_mcdm_' + str(i) + '.csv'
            print('- File: '+ fileName)            
            fileURI = experimentFolder + fileName
            distanceFileUri = experimentFolder + distanceFileName        
            dataIN = pd.read_csv(fileURI)
            distanceDataIn = pd.read_csv(distanceFileUri, index_col=False)
            dataIN["Experiment"] = experiment
            dataIN["Batch"] =  str(i)

            # Filter tags distances
            for i in range(1,11):
                distanceDataIn.loc[distanceDataIn['tag'+str(i)] > errorDist, 'tag'+str(i)] = errorDist

            # Stack the DataFrames on top of each other                
            if 'allData' in locals():
                allData = pd.concat([allData, dataIN], axis=0)
                distanceAllData = pd.concat([distanceAllData, distanceDataIn], axis=0)
            else:
                allData = dataIN
                distanceAllData = distanceDataIn


        # some data conditioning ...
        allData["Batch"] =  allData["Batch"].astype('category')
        allData["Experiment"] =  allData["Experiment"].astype('category')
        distanceAllData["Experiment"] =  allData["Experiment"].astype('category')

        grouping_colls = ['w_info_gain', 'w_travel_distance', 'w_sensing_time', 'w_rfid_gain', 'w_battery_status', 'Experiment']
        averages = allData.groupby(grouping_colls).mean() 
        averages2 = contitionDF(averages, 'av', None)
        distanceAverages = distanceAllData.groupby(grouping_colls).mean() 
        distanceAverages2 = contitionDF(distanceAverages, 'av', None)

        stdevs = allData.groupby(grouping_colls).std()
        distanceStdevs = distanceAllData.groupby(grouping_colls).std()
        droppedCols = ['Experiment', 'norm_w_info_gain', 'norm_w_travel_distance', 'norm_w_sensing_time', 'norm_w_rfid_gain', 'norm_w_battery_status', 'w_info_gain', 'w_travel_distance', 'w_sensing_time', 'w_rfid_gain', 'w_battery_status']
        distanceDroppedCols = ['tag1', 'tag2', 'tag3', 'tag4', 'tag5', 'tag6', 'tag7', 'tag8', 'tag9', 'tag10']
        stdevs2 = contitionDF(stdevs, 'std',droppedCols)
        stats = pd.concat([averages2, stdevs2], axis=1)
        distanceStdevs2 = contitionDF(distanceStdevs, 'std',distanceDroppedCols)
        distanceStats = pd.concat([distanceAverages2, distanceStdevs2], axis=1)
        
        # Calculate avg and std for tag distance
        distanceStats['tag_mean']=distanceStats.iloc[:,5:15].mean(axis=1)
        distanceStats['tag_std']=distanceStats.iloc[:,5:15].std(axis=1)

        # Round everything at 2 decimals
        stats = stats.round(2)
        distanceStats = distanceStats.round(2)

        # Merge av and std in the same column
        stats["batteryStatus"] = stats["batteryStatus (av)"].astype(str) + "(" + stats["batteryStatus (std)"].astype(str) + ")"
        stats["numConfiguration"] = stats["numConfiguration (av)"].astype(str) + "(" + stats["numConfiguration (std)"].astype(str) + ")"
        stats["travelledDistance"] = stats["travelledDistance (av)"].astype(str) + "(" + stats["travelledDistance (std)"].astype(str) + ")"
        stats["accuracy"] = stats["accuracy (av)"].astype(str) + "(" + stats["accuracy (std)"].astype(str) + ")"
        stats["coverage"] = stats["coverage (av)"].astype(str) + "(" + stats["coverage (std)"].astype(str) + ")"
        stats["totalScanTime"] = stats["totalScanTime (av)"].astype(str) + "(" + stats["totalScanTime (std)"].astype(str) + ")"
        stats["accumulatedRxPower"] = stats["accumulatedRxPower (av)"].astype(str) + "(" + stats["accumulatedRxPower (std)"].astype(str) + ")"

        # Merge avg and std in the same column
        distanceStats["tagAccuracy"] = distanceStats["tag_mean"].astype(str) + "(" + distanceStats["tag_std"].astype(str) + ")"
        
        # Remove useless column (without reassignment)
        column_to_remove = ["norm_w_info_gain (av)",
                            "norm_w_travel_distance (av)",
                            "norm_w_sensing_time (av)",
                            "norm_w_rfid_gain (av)",
                            "norm_w_battery_status (av)",
                            "coverage (av)",
                            "coverage (std)",
                            "numConfiguration (av)",
                            "numConfiguration (std)",
                            "travelledDistance (av)",
                            "travelledDistance (std)", 
                            "accuracy (av)",
                            "accuracy (std)", 
                            "totalScanTime (av)",
                            "totalScanTime (std)",
                            "accumulatedRxPower (av)",
                            "accumulatedRxPower (std)",
                            "batteryStatus (av)",
                            "batteryStatus (std)"]
        for column in column_to_remove:
            stats.drop(column, axis=1, inplace=True)

        # Remove useless column (without reassignment)
        distance_column_to_remove = ['tag1 (av)',
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
        for column in distance_column_to_remove:
            distanceStats.drop(column, axis=1, inplace=True)
        
        stats = stats.sort_values(by=["Experiment (av)", "w_info_gain (av)"])

        # Rearrange dataframe
        stats = stats[["Experiment (av)", 
                        "w_info_gain (av)",
                        "w_travel_distance (av)",
                        "w_sensing_time (av)",
                        "w_rfid_gain (av)",
                        "w_battery_status (av)",
                        "coverage",
                        "batteryStatus",
                        "numConfiguration",
                        "travelledDistance",
                        "accuracy",
                        "totalScanTime",
                        "accumulatedRxPower"]]

        stats_transposed = stats.T
        stats_transposed = stats_transposed.reindex(["w_info_gain (av)",
                                "w_travel_distance (av)",
                                "w_sensing_time (av)",
                                "w_rfid_gain (av)",
                                "w_battery_status (av)",
                                "Experiment (av)", 
                                "coverage",
                                "batteryStatus",
                                "numConfiguration",
                                "travelledDistance",
                                "accuracy",
                                "totalScanTime",
                                "accumulatedRxPower"])

        distance_stats_transposed = distanceStats.T
        # print(distance_stats_transposed[-1:])
        stats_transposed = pd.concat([stats_transposed, distance_stats_transposed[-1:]], axis=0)
        # print(stats_transposed[-1:])

        # Save to disk
        stats.to_csv('/home/pulver/Desktop/stat.csv', index=False)
        stats_transposed.to_csv('/home/pulver/Desktop/test_inb3123_new.csv', header=False)#, index=False)    
        distanceStats.to_csv('/home/pulver/Desktop/distance.csv', index=False)
        distance_stats_transposed.to_csv('/home/pulver/Desktop/distance_transposed.csv', header=False)#, index=False)

        
        finalData = pd.concat([finalData, stats_transposed], axis=0)



    # finalData.to_csv('/home/pulver/Desktop/final_weights.csv', index=True)

    # Drop any rows containing weights configuration
    mask = np.ones(len(finalData)).astype(bool)
    for i in range(1,len(experimentFolders)):
        mask[i*14:(i*14 + 5)] = False
    finalData = finalData[mask]
    finalData.to_csv('/home/pulver/Desktop/final_clean.csv', index=True)




# # In case you want to do some debugging ...
#  q = allData.query('w_info_gain==0.1 and w_travel_distance==0.6 and w_sensing_time==0.1 and w_rfid_gain==0.1 and w_battery_status==0.1')
#  print(q['travelledDistance'])
#  q = averages.query('w_info_gain==0.1 and w_travel_distance==0.6 and w_sensing_time==0.1 and w_rfid_gain==0.1 and w_battery_status==0.1')
#  print(q['travelledDistance'])

