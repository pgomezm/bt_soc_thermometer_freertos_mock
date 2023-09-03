********************************************************************************
********************************************************************************

# Instructions on using the commit template

The commit template forces the developer to a specific format.  The present 
document is intended as a guide on using it.

Know that all lines in the template that start with a hash (`#`) are comments.
They get stripped from the message automatically as part of the commit process;
thus, do not show in the commit message when looking back at the repository 
history/log.

## Commit Template

    # **** Include a 1-line description for the commit ending on a list of IDs in parentheses.
    #
    <Change Description> (#ID, #ID, #ID)                  **** <- UPDATE THIS LINE
    #
    # **** Include a list of all work items with a status [PENDING/ACTIVE/COMPLETE/REMOVED].
    ********************
    Work Items:
    * Task #0: Clean up code [COMPLETE]                   **** <- UPDATE THIS LINE
    #
    # **** Include a detailed list of all changes associated to each work item.
    ********************
    Details:
    * Task #0:                                            **** <- UPDATE THESE LINES
      * Removed function addTimeofDay from file ttrX.js.  **** <- UPDATE THESE LINES
      * Removed global variable my_ts from file ttrX.js.  **** <- UPDATE THESE LINES
    #
    # **** Include a list with any pertinent additional notes.
    ********************
    Notes:
    * The code compiles with 100 warnings.                **** <- UPDATE THESE LINES
    * Time reported is MST.                               **** <- UPDATE THESE LINES
    ********************
    # **** DO NOT ADD TEXT BELOW THIS LINE
    # **** DO NOT REMOVE THE LINES BELOW
    Commit template version 1.0.1
    ********************

## Sections

### 1-line description

    # **** Include a 1-line description for the commit ending on a list of IDs in parentheses.
    #
    <Change Description> (#ID, #ID, #ID)                  **** <- UPDATE THIS LINE

The first line (not including comments should be a 1-line description for the
commit.  It should end with a list in parentheses of each work item ID that is
addressed by the change.  **Only include the IDs for work items listed in the
details section.**

***MAKE SURE TO REMOVE THE `**** <- UPDATE THIS LINE` message!!!***

#### Format:

>\<Change Short Description\> (#ID, #ID, #ID)

#### E.g.:

>     Initialize Repository (#0)

>     Initialize Repository (#0, #12)

### List of work items

    #
    # **** Include a list of all work items with a status [PENDING/ACTIVE/COMPLETE/REMOVED].
    ********************
    Work Items:
    * Task #0: Clean up code [COMPLETE]                   **** <- UPDATE THIS LINE

The next section includes a list of all work items that are addressed by the
current change as well as all other changes that have taken place in the local
branch after the branch was created or last back merged from the main/TRUNK.
Only after the branch is merged to the main/TRUNK and upon the following back
merge should the list be reset to **`NONE`**.  **Commits to the branch should not
reset this list**.
For each work item, it's current status should also be included.

The section starts with a separator, a line with 20 asterisks (**`*`**).
It then includes the title for the section (**`Work Items:`**).
Right after the title a list of all work items should follow; each with a status
of **`PENDING`**, **`ACTIVE`**, **`COMPLETE`** or **`REMOVED`**.
If no work items are to be included, the list should explicitly indicate it via
the use of the keyword **`NONE`** as the sole item.

Valid work item types are: **`Epic`**, **`Feature`**, **`User Story`**, 
**`Task`**, **`Bug`**, etc. (*as per the avaiable work items types in DevOps*).
The work item ID must match the one in DevOps.

The name of the work item should ideally match the one given in DevOps.  Yet, it
could be shortened or simplified.  
The name of the work items should:
* Start always with a capital;
* Only include printable characters;
* Not include special characters such as tab (`\t`), bell (`\b`), newline
  (`\n`), carriage return (`\r`), or any non-printable or extended character;
* Not include forward slashes (`/`), backward slashes (`\`), any type of
  brackets (`{`, `}`, `[` or `]`) or parentheses (`(` or `)`);
* Not include a trailing period (`.`), comma (`,`), colon (`:`) or semicolon
  (`;`); and
* Not include unterminated backticks (`` ` ``), quotes (`'`), or double-quotes
  (`"`).

Each work item should include its status at the end of the line, in **CAPS** and
enclosed in square brackets (**`[`** and **`]`**).  The **ONLY** allowed values
for the status are:
* **`PENDING`**: Work for this work item is expected but it has not yet started.
* **`ACTIVE`**: Work has started for this work item but it is not complete.
* **`COMPLETE`**: All work needed for this work item has been completed.
* **`REMOVED`**:  Work for the work item was expected but the work item is no
longer to be included; any work already performed has been already removed.

Work items should be listed in incremental order based on their work item ID.

***MAKE SURE TO REMOVE ALL OF THE `**** <- UPDATE THIS LINE` messages!!!***

#### Format:

> \* \<Type\> #ID: \<Work Item Title\> [PENDING/ACTIVE/COMPLETE/REMOVED]  
> or  
> \* NONE

For each work item there should be a line with:
* A leading asterisk (`*`) followed by a space; 
* The work item type followed by one or more spaces - make it so the hashes
  (`#`) that follow end up aligned;
* A hash (`#`) followed by the work item ID, a colon (`:`) and one or more
spaces - make it so the names of the work items end up aligned;
* The name of the work item followed by a space; and
* The status of the work item enclosed in square brackets (**`[`** and **`]`**).

If no work items are to be included, the list should explicitly indicate it via
the use of the keyword **`NONE`** as the sole item.

#### E.g.:

>     * Task #31:  Modify sensor logic to track sensor reset messages [PENDING]

>     * Bug  #23:  Fix low pass filter in reading processing [PENDING]
>     * Task #31:  Modify sensor logic to track sensor reset messages [ACTIVE]
>     * Task #101: Clean up code [COMPLETE]
>     * Task #103: Change all tags to french [REMOVED]

>     * NONE

### Details for each work item

    #
    # **** Include a detailed list of all changes associated to each work item.
    ********************
    Details:
    * Task #0:                                            **** <- UPDATE THESE LINES
      * Removed function addTimeofDay from file ttrX.js.  **** <- UPDATE THESE LINES
      * Removed global variable my_ts from file ttrX.js.  **** <- UPDATE THESE LINES

The next section includes a detailed list of all changes done since the last
commit for each work item.  Any changes not associated with a particular work
item to be listed under a generic **`Other`** category at the end of the list.
The **`Other`** group should only exist if there are any changes to be listed
against it (same applies to the work items).

The list of specific details for each work item (or **`Other`**) is specific to
each commit.  A copy of these is expected to be maintained in the changelog
file (`docs\changelog.md`) for the project.

If no changes have been made against a particular work item ID, then that work
item ID should not be listed in the list of details (but still included in the
list of work items as described above); yet, if no change is made for any work
item nor any change that would be listed in the **`Other`** group, then and
only then should the list of details include a single line with an explicit
**`NONE`**.

***MAKE SURE TO REMOVE ALL OF THE `**** <- UPDATE THESE LINES` messages!!!***

#### Format:

> \* \<Type\> #ID:  
>   \* \<Description of a change\>  
>   \* \<Description of another change\>  
>   **...**  
> or  
> \* \<Type\> #ID:  
>   \* \<Description of a change\>  
>     \<Subsequent line\>  
>     \<Subsequent line\>  
>   \* \<Description of another change\>  
>     \<Subsequent line\>  
>     \* \<Additional details for the description\>  
>       \<Subsequent line\>  
>   **...**  
> or  
> \* NONE

For each work item there should be a line with:

* A leading asterisk (`*`) followed by a space; 
* The work item type followed by one space;
* A hash (`#`) followed by the work item ID and a colon (`:`).

Then the list of details for that particular work item should follow.
Each detail in one or more lines where:

* Each detail starts on a new line;
* The first line of the description having two (2) leading spaces;
* Then an asterisk (`*`) followed by a space;
* Last the description.
  * If the description takes more than one line, each succesive line should
    start with four (4) leading spaces.
  * Details for the details to start on a new line with four (4) leading spaces;
  * Then an asterisk (`*`) followed by a space;
  * Last the details of the details;
  * Subsequent lines staring with six (6) leading spaces.
  * Details for the details of the details to start on a new line with six (6)
    leading spaces;
  * Then an asterisk (`*`) followed by a space;
  * Last the details for the details of the details;
  * Subsequent lines staring with eight (8) leading spaces.
  * ... and so it goes.

#### E.g.:

>    \* Task #31:  
>      \* Modified method decodeMessages in class BlahBlahSensor to detect reset  
>        messages and call the appropriate decoder.  
>      \* Added function decodeBlahBlahSensosor_resetMessage to file  
>        BlahBlahSensor_decoder.js with code to decode the fields in the message.  
>        \* Added function decodeBlahBlahSensosor_resetMessage to file  
>        \* Added function decodeBlahBlahSensosor_resetMessage to file  
>          BlahBlahSensor_decoder.js with code to decode the fields in the message.  
>        \* Added function decodeBlahBlahSensosor_resetMessage to file  
>          \* Added function decodeBlahBlahSensosor_resetMessage to file  
>            BlahBlahSensor_decoder.js with code to decode the fields in the message.  
>          \* Added function decodeBlahBlahSensosor_resetMessage to file  

>    \* Task #31:  
>      \* Modified method decodeMessages in class BlahBlahSensor to detect reset  
>        messages and call the appropriate decoder.  
>      \* Added function decodeBlahBlahSensosor_resetMessage to file  
>        BlahBlahSensor_decoder.js with code to decode the fields in the message.  
>    \* Task #101:  
>      \* Removed function addTimeofDay (OBSOLETE) from file ttrX.js.  
>      \* Removed global variable my_ts (NOT USED) from from file ttrX.js.  
>      \* Added comments to function cleanScreen in file damian.js to clarify the  
>        process.  
>      \* Renamed method decodeMsgT1 to decodeInitMsg in class mySensor for clarity.  
>    \* Other:  
>      \* Updated copyright notice in all files to 2023.  

>    \* Other:  
>      \* Updated copyright notice in all files to 2023.

>    \* NONE

### Additional Notes

    #
    # **** Include a list with any pertinent additional notes.
    ********************
    Notes:
    * The code compiles with 100 warnings.                **** <- UPDATE THESE LINES
    * Time reported is MST.                               **** <- UPDATE THESE LINES

The next section includes a list with additional notes that could be of value
for any developer looking at the code.

It is critical to include a list of compilation errors; ideally no should 
exist.  Also a list of all warnings.  
If for whatever reason there are errors and/or warnings, not only include them
but comment on how to deal with them or the reason why they could safely be
ignored.  
***NOTE: ERRORS AND/OR WARNING SHOULD NOT EXIST OR SHOULD BE PROPERLY JUSTIFIED***

Also, mention of any irregularities in the  committed code that should be taken
in consideration.

If there are no signfificant notes to be mentioned, the section should
explicitly indicate it via the use of the keyword **`NONE`** as the sole item.

***MAKE SURE TO REMOVE ALL OF THE `**** <- UPDATE THESE LINES` messages!!!***

#### Format:

> \* \<Description of a note\>  
> \* \<Description of another note\>  
> **...**  
> or  
> \* \<Description of a note\>  
>   \<Subsequent line\>  
>   \<Subsequent line\>  
> \* \<Description of another note\>  
>   \<Subsequent line\>  
>   \* \<Additional details for the note\>  
>     \<Subsequent line\>  
> **...**  
> or  
> \* NONE

For each note there its important to:

* Start each note on a new line;
* The first line of the note having a leading asterisk (`*`) followed by a space;
* The the actual note.
  * If the note takes more than one line, each succesive line should
    start with two (2) leading spaces.
  * Details for the note to start on a new line with two (2) leading spaces;
  * Then an asterisk (`*`) followed by a space;
  * Last the details of the note;
  * Subsequent lines staring with four (4) leading spaces.
  * Details for the details of the note to start on a new line with four (4)
    leading spaces;
  * Then an asterisk (`*`) followed by a space;
  * Last the details for the details of the note;
  * Subsequent lines staring with six (6) leading spaces.
  * ... and so it goes.

#### E.g.:

> \* Commit on 15-May-2023 backdated to date of original change.  
> \* Change previously referenced to as CC-00001305: Validate access to sensor  
>   data from python script (1d)  
> \* Released as **Version *V1***

> \* Commit on 15-May-2023 backdated to date of original change.

> \* NONE

### Additional Notes

    ********************
    # **** DO NOT ADD TEXT BELOW THIS LINE
    # **** DO NOT REMOVE THE LINES BELOW
    Commit template version 1.0.1
    ********************

The last section **SHOULD NOT BE MODIFIED**.  It includes a separator line,
A couple of comments explicitly indicating not to modify these lines nor 
writting anything after them, the version of the commit template and a final
separator line.

**DO NOT MODIFY ANYTHING IN THIS SECTION**

This last section is intended to identify the format of the commit message as
following the particular version of the commit template.  This information is
intended for their eventual use in automatic processing of the commit message
by local and server hooks.

********************************************************************************
********************************************************************************