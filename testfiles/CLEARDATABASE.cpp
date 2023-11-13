// /usr/bin/g++ -g CLEARDATABASE.cpp -o CLEARDATABASE -lmysqlcppconn

#include <iostream>
#include <mysql_driver.h>
#include <mysql_connection.h>
#include <cppconn/statement.h>

int main() {
    // MySQL database connection settings
    sql::mysql::MySQL_Driver *driver;
    sql::Connection *con;

    // Include the database name in the connection string
    driver = sql::mysql::get_mysql_driver_instance();
    con = driver->connect("tcp://127.0.0.1:3306/sensor_data", "kippenbroeder", "kippenbroederinator");

    // Execute DELETE query to clear all data from the table
    try {
        sql::Statement *stmt = con->createStatement();
        stmt->execute("DELETE FROM data");
        delete stmt;

        std::cout << "All data deleted from data table." << std::endl;
    } catch (sql::SQLException &e) {
        std::cerr << "MySQL Error: " << e.what() << std::endl;
    }

    delete con;

    return 0;
}
