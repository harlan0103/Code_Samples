Name: Haoran Liang

Directory structure:
---- headers
	 | ---- MyDB_Page.h
	 | ---- MyDB_PageHandle.h
	 | ---- MyDB_LRU.h
	 | ---- MyDB_BufferManager.h
---- source
	 | ---- MyDB_Page.cc
	 | ---- MyDB_PageHandle.cc
	 | ---- MyDB_LRU.cc
	 | ---- MyDB_BufferManager.cc
---- README.md

Description:
This code sample is from a course project I was written at Rice University. The goal for this code sample is to build a LRU buffer manager. This buffer manager is responsible for managing a pool of pages that are going to be used to buffer temporary data, and that will be used to buffer pages from database tables. This buffer manager serves as a foundation of the entire database system I implemented in that course.

--- MyDB_Page.cc
This file contains the structure for a page. A page has reference counter and is abled to be written bytes. There are two type of pages. Pages that are associated with some position in actual database table, and pages that are anonymous and are used as temporary storage. All anonymous pages are mapped to slots in a single temporary file. Pages can also be pinned. A pinned page is always kept in RAM, and allocating a pinned page will take a page out of the buffer pool.

--- MyDB_PageHandle.cc
This database system uses Page Handle to access pages. Page handle can read the content from pages and write bytes to pages. Page handels are implemented via C++ smart pointers.

--- MyDB_LRU.cc
The implementation of LRU algorithm. This LRU algorithm uses a double linked list to hold nodes. Node structure contains pages, and has next and previous pointer. The most recent used node will be moved to the front of the list, and tail has the least used node. When a new node is inserted into the double linked list, the least used node will be poped out. This algorithm ensures that the number of pages will not exceed the size of the cache.

--- MyDB_BufferManager.cc
An interface that provides functions for the database system to create buffer manager and interact with pages

This code sample was written on: 2019.01.28




