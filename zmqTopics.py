topic2portDict = {}
       
topicMavlinkPort             = 7790
topicMavlinkFlightData       = b'FLIGHT_DATA'
topic2portDict[topicMavlinkFlightData]       = topicMavlinkPort

topicGuidenceCmdPort            = 7793
topicGuidenceCmdAttitude        = b'quadAttitudeCmd'
topicGuidenceCmdVelNed          = b'quadVelNedCmd'
topicGuidenceCmdVelBody         = b'quadVelBodyCmd'
topicGuidenceCmdAcc             = b'quadAccCmd'
topicGuidenceCmdTakeoff         = b'quadTakeoffCmd'
topicGuidenceCmdLand            = b'quadLandCmd'
topicGuidanceCmdArm             = b'quadArmCmd'
topic2portDict[topicGuidenceCmdAttitude] = topicGuidenceCmdPort
topic2portDict[topicGuidenceCmdVelNed] = topicGuidenceCmdPort
topic2portDict[topicGuidenceCmdVelBody] = topicGuidenceCmdPort
topic2portDict[topicGuidenceCmdAcc] = topicGuidenceCmdPort
topic2portDict[topicGuidenceCmdTakeoff] = topicGuidenceCmdPort
topic2portDict[topicGuidenceCmdLand] = topicGuidenceCmdPort
topic2portDict[topicGuidanceCmdArm] = topicGuidenceCmdPort



