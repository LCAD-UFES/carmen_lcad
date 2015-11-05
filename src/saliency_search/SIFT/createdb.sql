GRANT ALL PRIVILEGES ON *.* TO 'neobrain'@'localhost' IDENTIFIED BY '12341234' WITH GRANT OPTION;

create table SiftKeypoints (kid integer, value blob);

create table SiftKeypoints (
  kid integer unsigned NOT NULL AUTO_INCREMENT,
  value blob NOT NULL,
  matchCount integer unsigned default 1,
  Primary Key (kid)
);


delimiter //

CREATE FUNCTION nearest_neighbor(kpValue blob) 
RETURNS INT DETERMINISTIC 
BEGIN
   DECLARE kid INT;
   DECLARE distOut REAL;
   select sk.kid, kp_dist(kpvalue, sk.value) as dist INTO kid, distOut 
    FROM SiftKeypoints sk order by dist limit 1;
    RETURN kid;
END;

//

delimiter ;

delimiter //

CREATE FUNCTION nearest_neighbor2(kpValue blob)
RETURNS INT DETERMINISTIC
BEGIN
  DECLARE nkid INT;
  DECLARE distOut REAL;
  DECLARE skmid INT;

  select kmeans_nearest_neighbor(kpValue) into skmid;

  select sk.kid, kp_dist(kpValue, sk.value) as dist into nkid, distOut
    from SiftKeypoints sk,
     (select c.kid from centers c where c.kmid = skmid) mkids
    where sk.kid = mkids.kid
    order by dist limit 1;
  RETURN nkid;
END;

//

delimiter ;






delimiter //

CREATE FUNCTION kmeans_nearest_neighbor(kpValue blob) 
RETURNS INT DETERMINISTIC 
BEGIN
   DECLARE kid INT;
   DECLARE distOut REAL;
   select km.kmid, kp_dist(kpvalue, km.value) as dist INTO kid, distOut 
    FROM kmeans km order by dist limit 1;
    RETURN kid;
END;

//

delimiter ;




CREATE TABLE kmeans ( 
  kmid integer unsigned NOT NULL AUTO_INCREMENT,
  value blob,
  Primary Key (kmid));


CREATE TABLE centers (kmid INTEGER, kid INTEGER);


k=1000
Get the initial centers
INSERT INTO kmeans
SELECT value FROM SiftKeypoints ORDER BY Rand() LIMIT 1000;

Repeat
Foreach object, find its closest center

select km.kid, kp_dist(sk.value, km.value) as dist from SiftKeypoints sk, kmeans km where sk.kid = 1 order by dist limit 1;

INSERT INTO centers
select kmeans_nearest_neighbor(value) as kmid, kid from SiftKeypoints;

Get the new centers
Find the mean of each center and its assigned memebers

Repeat Untill stable (no objects have moved)

//Kmid should be in SiftKeypoints
// To find the Nearset neigbor
//FInd the value closes in the kmeans
select kid, kmeans_nearest_neighbor(value) from SiftKeypoints where kid=1


select sk.kid from SiftKeypoints sk, (select kid from centers where kmid = 97) mkids where sk.kid = mkids.kid;




//FOr testing
select value into @kp from SiftKeypoints where kid=10;

