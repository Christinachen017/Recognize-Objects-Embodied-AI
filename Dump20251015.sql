CREATE DATABASE  IF NOT EXISTS `united_helpers` /*!40100 DEFAULT CHARACTER SET utf8mb4 COLLATE utf8mb4_0900_ai_ci */ /*!80016 DEFAULT ENCRYPTION='N' */;
USE `united_helpers`;
-- MySQL dump 10.13  Distrib 8.0.43, for macos15 (arm64)
--
-- Host: localhost    Database: united_helpers
-- ------------------------------------------------------
-- Server version	8.0.43

/*!40101 SET @OLD_CHARACTER_SET_CLIENT=@@CHARACTER_SET_CLIENT */;
/*!40101 SET @OLD_CHARACTER_SET_RESULTS=@@CHARACTER_SET_RESULTS */;
/*!40101 SET @OLD_COLLATION_CONNECTION=@@COLLATION_CONNECTION */;
/*!50503 SET NAMES utf8 */;
/*!40103 SET @OLD_TIME_ZONE=@@TIME_ZONE */;
/*!40103 SET TIME_ZONE='+00:00' */;
/*!40014 SET @OLD_UNIQUE_CHECKS=@@UNIQUE_CHECKS, UNIQUE_CHECKS=0 */;
/*!40014 SET @OLD_FOREIGN_KEY_CHECKS=@@FOREIGN_KEY_CHECKS, FOREIGN_KEY_CHECKS=0 */;
/*!40101 SET @OLD_SQL_MODE=@@SQL_MODE, SQL_MODE='NO_AUTO_VALUE_ON_ZERO' */;
/*!40111 SET @OLD_SQL_NOTES=@@SQL_NOTES, SQL_NOTES=0 */;

--
-- Table structure for table `assignment`
--

DROP TABLE IF EXISTS `assignment`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
CREATE TABLE `assignment` (
  `assignmentId` int NOT NULL AUTO_INCREMENT,
  `volunteerId` int DEFAULT NULL,
  `taskId` int DEFAULT NULL,
  `startDateTime` datetime DEFAULT NULL,
  `endDateTime` datetime DEFAULT NULL,
  PRIMARY KEY (`assignmentId`),
  KEY `volunteerId` (`volunteerId`),
  KEY `taskId` (`taskId`),
  CONSTRAINT `assignment_ibfk_1` FOREIGN KEY (`volunteerId`) REFERENCES `volunteer` (`volunteerId`) ON DELETE CASCADE ON UPDATE CASCADE,
  CONSTRAINT `assignment_ibfk_2` FOREIGN KEY (`taskId`) REFERENCES `task` (`taskId`) ON DELETE CASCADE ON UPDATE CASCADE
) ENGINE=InnoDB AUTO_INCREMENT=15 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `assignment`
--

LOCK TABLES `assignment` WRITE;
/*!40000 ALTER TABLE `assignment` DISABLE KEYS */;
INSERT INTO `assignment` VALUES (1,1,1,'2025-09-01 08:00:00','2025-12-31 16:00:00'),(2,1,2,'2025-09-10 08:00:00','2025-09-30 12:00:00'),(3,1,6,'2025-09-06 09:00:00','2025-09-30 13:00:00'),(4,2,2,'2025-09-11 09:00:00','2025-09-25 14:00:00'),(5,2,4,'2025-09-12 10:00:00','2025-09-20 13:00:00'),(6,3,3,'2025-09-15 07:00:00','2025-09-15 19:00:00'),(7,4,5,'2025-09-13 06:30:00','2025-09-18 18:00:00'),(8,5,8,'2025-09-21 08:00:00','2025-09-28 12:00:00'),(9,6,9,'2025-09-25 09:00:00','2025-10-05 17:00:00'),(10,7,10,'2025-09-08 08:30:00','2025-09-12 12:00:00'),(11,8,4,'2025-09-23 10:00:00','2025-09-30 15:00:00'),(12,9,2,'2025-09-12 08:00:00','2025-09-22 12:00:00'),(13,10,4,'2025-09-13 11:00:00','2025-09-20 14:00:00'),(14,5,2,'2025-09-22 09:00:00','2025-10-01 13:00:00');
/*!40000 ALTER TABLE `assignment` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `item`
--

DROP TABLE IF EXISTS `item`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
CREATE TABLE `item` (
  `itemId` int NOT NULL AUTO_INCREMENT,
  `itemDescription` varchar(200) NOT NULL,
  `itemValue` decimal(8,2) NOT NULL,
  `quantityOnHand` int NOT NULL,
  PRIMARY KEY (`itemId`)
) ENGINE=InnoDB AUTO_INCREMENT=16 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `item`
--

LOCK TABLES `item` WRITE;
/*!40000 ALTER TABLE `item` DISABLE KEYS */;
INSERT INTO `item` VALUES (1,'Antiseptic wipe packet',0.25,1200),(2,'Adhesive bandage',0.10,5000),(3,'Disposable gloves',0.50,800),(4,'Antibiotic ointment (tube)',1.50,300),(5,'Diaper',0.40,1500),(6,'Small plush toy',2.50,600),(7,'Canned meal',1.75,4000),(8,'Protein bar',1.20,3500),(9,'Juice box',0.90,2000),(10,'Soap bar',0.60,1800),(11,'Toothbrush',0.80,2200),(12,'Toothpaste',1.10,1500),(13,'Small towel',3.00,400),(14,'Blanket',8.50,250),(15,'Surgical mask',0.20,3000);
/*!40000 ALTER TABLE `item` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `package`
--

DROP TABLE IF EXISTS `package`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
CREATE TABLE `package` (
  `packageId` int NOT NULL AUTO_INCREMENT,
  `packageCreateDate` date NOT NULL,
  `packageWeight` decimal(6,2) NOT NULL,
  `taskId` int DEFAULT NULL,
  `packageTypeId` int DEFAULT NULL,
  PRIMARY KEY (`packageId`),
  KEY `taskId` (`taskId`),
  KEY `packageTypeId` (`packageTypeId`),
  CONSTRAINT `package_ibfk_1` FOREIGN KEY (`taskId`) REFERENCES `task` (`taskId`) ON DELETE CASCADE ON UPDATE CASCADE,
  CONSTRAINT `package_ibfk_2` FOREIGN KEY (`packageTypeId`) REFERENCES `package_type` (`packageTypeId`) ON DELETE RESTRICT ON UPDATE CASCADE
) ENGINE=InnoDB AUTO_INCREMENT=13 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `package`
--

LOCK TABLES `package` WRITE;
/*!40000 ALTER TABLE `package` DISABLE KEYS */;
INSERT INTO `package` VALUES (1,'2025-09-11',2.50,2,3),(2,'2025-09-11',2.70,2,3),(3,'2025-09-12',2.60,2,3),(4,'2025-09-13',2.55,2,3),(5,'2025-09-13',2.65,2,3),(6,'2025-09-14',0.85,4,1),(7,'2025-09-14',0.88,4,1),(8,'2025-09-14',0.90,4,1),(9,'2025-09-21',1.20,8,2),(10,'2025-09-22',1.25,8,2),(11,'2025-09-23',3.50,4,4),(12,'2025-09-24',3.60,4,4);
/*!40000 ALTER TABLE `package` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `package_contents`
--

DROP TABLE IF EXISTS `package_contents`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
CREATE TABLE `package_contents` (
  `packageContentsId` int NOT NULL AUTO_INCREMENT,
  `packageId` int DEFAULT NULL,
  `itemId` int DEFAULT NULL,
  `itemQuantity` int NOT NULL,
  PRIMARY KEY (`packageContentsId`),
  KEY `packageId` (`packageId`),
  KEY `itemId` (`itemId`),
  CONSTRAINT `package_contents_ibfk_1` FOREIGN KEY (`packageId`) REFERENCES `package` (`packageId`) ON DELETE CASCADE ON UPDATE CASCADE,
  CONSTRAINT `package_contents_ibfk_2` FOREIGN KEY (`itemId`) REFERENCES `item` (`itemId`) ON DELETE RESTRICT ON UPDATE CASCADE
) ENGINE=InnoDB AUTO_INCREMENT=41 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `package_contents`
--

LOCK TABLES `package_contents` WRITE;
/*!40000 ALTER TABLE `package_contents` DISABLE KEYS */;
INSERT INTO `package_contents` VALUES (1,1,7,3),(2,1,8,2),(3,1,9,1),(4,2,7,3),(5,2,8,2),(6,2,9,1),(7,3,7,3),(8,3,8,2),(9,3,9,1),(10,4,7,3),(11,4,8,2),(12,4,9,1),(13,5,7,3),(14,5,8,2),(15,5,9,1),(16,6,1,2),(17,6,2,5),(18,6,3,1),(19,6,4,1),(20,7,1,2),(21,7,2,5),(22,7,3,1),(23,8,1,3),(24,8,2,6),(25,8,3,1),(26,8,4,1),(27,9,5,2),(28,9,6,1),(29,9,9,1),(30,10,5,2),(31,10,6,1),(32,10,11,1),(33,11,10,1),(34,11,11,1),(35,11,12,1),(36,11,13,1),(37,12,10,1),(38,12,11,1),(39,12,12,1),(40,12,14,1);
/*!40000 ALTER TABLE `package_contents` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `package_type`
--

DROP TABLE IF EXISTS `package_type`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
CREATE TABLE `package_type` (
  `packageTypeId` int NOT NULL AUTO_INCREMENT,
  `packageTypeName` varchar(50) NOT NULL,
  PRIMARY KEY (`packageTypeId`),
  UNIQUE KEY `packageTypeName` (`packageTypeName`)
) ENGINE=InnoDB AUTO_INCREMENT=6 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `package_type`
--

LOCK TABLES `package_type` WRITE;
/*!40000 ALTER TABLE `package_type` DISABLE KEYS */;
INSERT INTO `package_type` VALUES (1,'basic_medical'),(5,'blanket'),(2,'child_care'),(3,'food'),(4,'hygiene');
/*!40000 ALTER TABLE `package_type` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `packing_list`
--

DROP TABLE IF EXISTS `packing_list`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
CREATE TABLE `packing_list` (
  `packingListId` int NOT NULL AUTO_INCREMENT,
  `packingListName` varchar(100) NOT NULL,
  `packingListDescription` text,
  PRIMARY KEY (`packingListId`)
) ENGINE=InnoDB AUTO_INCREMENT=5 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `packing_list`
--

LOCK TABLES `packing_list` WRITE;
/*!40000 ALTER TABLE `packing_list` DISABLE KEYS */;
INSERT INTO `packing_list` VALUES (1,'basic_medical_kit','Wipes, bandages, gloves, ointment.'),(2,'child_care_kit','Diapers, toys, baby wipes, feeding spoon.'),(3,'food_basic_kit','Canned meals, protein bars, juice box, meat.'),(4,'hygiene_kit','Soap, toothbrush, toothpaste, towel.');
/*!40000 ALTER TABLE `packing_list` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `task`
--

DROP TABLE IF EXISTS `task`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
CREATE TABLE `task` (
  `taskId` int NOT NULL AUTO_INCREMENT,
  `taskCode` varchar(20) NOT NULL,
  `taskDescription` text,
  `taskTypeId` int DEFAULT NULL,
  `taskStatusId` int DEFAULT NULL,
  `packingListId` int DEFAULT NULL,
  PRIMARY KEY (`taskId`),
  UNIQUE KEY `taskCode` (`taskCode`),
  KEY `taskTypeId` (`taskTypeId`),
  KEY `taskStatusId` (`taskStatusId`),
  KEY `packingListId` (`packingListId`),
  CONSTRAINT `task_ibfk_1` FOREIGN KEY (`taskTypeId`) REFERENCES `task_type` (`taskTypeId`) ON DELETE RESTRICT ON UPDATE CASCADE,
  CONSTRAINT `task_ibfk_2` FOREIGN KEY (`taskStatusId`) REFERENCES `task_status` (`taskStatusId`) ON DELETE RESTRICT ON UPDATE CASCADE,
  CONSTRAINT `task_ibfk_3` FOREIGN KEY (`packingListId`) REFERENCES `packing_list` (`packingListId`) ON DELETE SET NULL ON UPDATE CASCADE
) ENGINE=InnoDB AUTO_INCREMENT=11 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `task`
--

LOCK TABLES `task` WRITE;
/*!40000 ALTER TABLE `task` DISABLE KEYS */;
INSERT INTO `task` VALUES (1,'T101','Answer incoming calls and log disaster reports',2,1,NULL),(2,'T102','Prepare 5,000 food packages for shelters',1,1,3),(3,'T103','Transport donated supplies to regional warehouse',3,2,NULL),(4,'T104','Assemble 2,000 basic medical kits for clinics',1,2,1),(5,'T105','Set up intake station at community center',3,2,NULL),(6,'T106','Data entry and update of item inventory',4,1,NULL),(7,'T107','Coordinate mobile medical support team',5,2,NULL),(8,'T108','Assemble 500 child-care packages',1,1,2),(9,'T109','Distribute hygiene kits at shelters',3,2,NULL),(10,'T110','Inventory audit of main warehouse',4,1,NULL);
/*!40000 ALTER TABLE `task` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `task_status`
--

DROP TABLE IF EXISTS `task_status`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
CREATE TABLE `task_status` (
  `taskStatusId` int NOT NULL AUTO_INCREMENT,
  `taskStatusName` varchar(50) NOT NULL,
  PRIMARY KEY (`taskStatusId`),
  UNIQUE KEY `taskStatusName` (`taskStatusName`)
) ENGINE=InnoDB AUTO_INCREMENT=5 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `task_status`
--

LOCK TABLES `task_status` WRITE;
/*!40000 ALTER TABLE `task_status` DISABLE KEYS */;
INSERT INTO `task_status` VALUES (4,'cancelled'),(3,'completed'),(2,'ongoing'),(1,'open');
/*!40000 ALTER TABLE `task_status` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `task_type`
--

DROP TABLE IF EXISTS `task_type`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
CREATE TABLE `task_type` (
  `taskTypeId` int NOT NULL AUTO_INCREMENT,
  `taskTypeName` varchar(50) NOT NULL,
  PRIMARY KEY (`taskTypeId`),
  UNIQUE KEY `taskTypeName` (`taskTypeName`)
) ENGINE=InnoDB AUTO_INCREMENT=6 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `task_type`
--

LOCK TABLES `task_type` WRITE;
/*!40000 ALTER TABLE `task_type` DISABLE KEYS */;
INSERT INTO `task_type` VALUES (4,'administrative'),(3,'logistics'),(5,'medical'),(1,'packing'),(2,'recurring');
/*!40000 ALTER TABLE `task_type` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `volunteer`
--

DROP TABLE IF EXISTS `volunteer`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
CREATE TABLE `volunteer` (
  `volunteerId` int NOT NULL AUTO_INCREMENT,
  `volunteerName` varchar(50) NOT NULL,
  `volunteerAddress` varchar(100) DEFAULT NULL,
  `volunteerTelephone` varchar(30) DEFAULT NULL,
  PRIMARY KEY (`volunteerId`)
) ENGINE=InnoDB AUTO_INCREMENT=11 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `volunteer`
--

LOCK TABLES `volunteer` WRITE;
/*!40000 ALTER TABLE `volunteer` DISABLE KEYS */;
INSERT INTO `volunteer` VALUES (1,'Maria','12 Maple St, Albany NY','111-333-0101'),(2,'James','99 Pine Ave, Schenectady NY','222-555-0102'),(3,'Aisha','45 Oak Rd, Troy NY','333-444-0103'),(4,'Lia','77 Elm St, Albany NY','111-555-0104'),(5,'Lucy','201 River Rd, Cohoes NY','666-555-0105'),(6,'Olivia','5 Birch Ln, Albany NY','777-555-0106'),(7,'Noah','8 Cedar Dr, Schenectady NY','888-111-0107'),(8,'Emma','30 Spruce St, Troy NY','555-111-0108'),(9,'Daniel','14 Willow Ave, Albany NY','999-222-0109'),(10,'Sophia','220 Market St, Cohoes NY','333-111-0110');
/*!40000 ALTER TABLE `volunteer` ENABLE KEYS */;
UNLOCK TABLES;
/*!40103 SET TIME_ZONE=@OLD_TIME_ZONE */;

/*!40101 SET SQL_MODE=@OLD_SQL_MODE */;
/*!40014 SET FOREIGN_KEY_CHECKS=@OLD_FOREIGN_KEY_CHECKS */;
/*!40014 SET UNIQUE_CHECKS=@OLD_UNIQUE_CHECKS */;
/*!40101 SET CHARACTER_SET_CLIENT=@OLD_CHARACTER_SET_CLIENT */;
/*!40101 SET CHARACTER_SET_RESULTS=@OLD_CHARACTER_SET_RESULTS */;
/*!40101 SET COLLATION_CONNECTION=@OLD_COLLATION_CONNECTION */;
/*!40111 SET SQL_NOTES=@OLD_SQL_NOTES */;

-- Dump completed on 2025-10-16  0:19:35
