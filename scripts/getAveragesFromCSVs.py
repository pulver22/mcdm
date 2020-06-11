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
    experimentFolders = ['inb3123', 'inbatrium', 'inbeng', 'ncfm', 'orebro' ]
    # experimentFolders = ['inb3123', 'ncfm', 'orebro' ]
    # Experiment batches 
    numBatches = 3 

    # some debug output
    print('Using data folder: ['+ dataFolder + ']')
    print('Experiments: '+ '\n\t- ' + '\n\t- '.join(experimentFolders))

    # just in case you are copypasting ...
    if 'allData' in locals():
        del allData

    for experiment in experimentFolders:      
        print('- Processing: '+ experiment)
        # experiment = experimentFolders[0] 
        experimentFolder = dataFolder + experiment + '/'
        for i in range(0,numBatches):
            print('- Batch: '+ str(i))
            # i = 0
            # FIXME: careful with number padding (01 or just 1?)
            fileName = 'result_' + experiment + '_mcdm_r'+ str(i) + '.csv'
            print('- File: '+ fileName)            
            fileURI = experimentFolder + fileName          
            dataIN = pd.read_csv(fileURI)
            dataIN["Experiment"] = experiment
            dataIN["Batch"] =  str(i)

            # Stack the DataFrames on top of each other                
            if 'allData' in locals():
                allData = pd.concat([allData, dataIN], axis=0)
            else:
                allData = dataIN


        # some data conditioning ...
        allData["Batch"] =  allData["Batch"].astype('category')
        allData["Experiment"] =  allData["Experiment"].astype('category')    

        grouping_colls = ['w_info_gain', 'w_travel_distance', 'w_sensing_time', 'w_rfid_gain', 'w_battery_status', 'Experiment']
        averages = allData.groupby(grouping_colls).mean() 
        averages2 = contitionDF(averages, 'av', None)

        stdevs = allData.groupby(grouping_colls).std() 
        droppedCols = ['Experiment', 'norm_w_info_gain', 'norm_w_travel_distance', 'norm_w_sensing_time', 'norm_w_rfid_gain', 'norm_w_battery_status', 'w_info_gain', 'w_travel_distance', 'w_sensing_time', 'w_rfid_gain', 'w_battery_status']
       
        stdevs2 = contitionDF(stdevs, 'std',droppedCols)
        stats = pd.concat([averages2, stdevs2], axis=1)
        
        # Round everything at 2 decimals
        stats = stats.round(2)

        # Merge av and std in the same column
        stats["batteryStatus"] = stats["batteryStatus (av)"].astype(str) + "(" + stats["batteryStatus (std)"].astype(str) + ")"
        stats["numConfiguration"] = stats["numConfiguration (av)"].astype(str) + "(" + stats["numConfiguration (std)"].astype(str) + ")"
        stats["travelledDistance"] = stats["travelledDistance (av)"].astype(str) + "(" + stats["travelledDistance (std)"].astype(str) + ")"
        stats["accuracy"] = stats["accuracy (av)"].astype(str) + "(" + stats["accuracy (std)"].astype(str) + ")"
        stats["coverage"] = stats["coverage (av)"].astype(str) + "(" + stats["coverage (std)"].astype(str) + ")"
        stats["totalScanTime"] = stats["totalScanTime (av)"].astype(str) + "(" + stats["totalScanTime (std)"].astype(str) + ")"
        stats["accumulatedRxPower"] = stats["accumulatedRxPower (av)"].astype(str) + "(" + stats["accumulatedRxPower (std)"].astype(str) + ")"
        
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

    # Save to disk
    stats.to_csv('/home/pulver/Desktop/test_average_stats_3.csv', index=False)

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
    stats_transposed.to_csv('/home/pulver/Desktop/test_final.csv', header=False)#, index=False)





# # In case you want to do some debugging ...
#  q = allData.query('w_info_gain==0.1 and w_travel_distance==0.6 and w_sensing_time==0.1 and w_rfid_gain==0.1 and w_battery_status==0.1')
#  print(q['travelledDistance'])
#  q = averages.query('w_info_gain==0.1 and w_travel_distance==0.6 and w_sensing_time==0.1 and w_rfid_gain==0.1 and w_battery_status==0.1')
#  print(q['travelledDistance'])

