import * as mongodb from 'mongodb';
import { Service } from 'typedi';
import { DB_URL_CONNEXTION, DB_DATABASE_NAME } from '../constants/data-base-constants';


@Service()
export class DataBaseHandler {
  private client: mongodb.MongoClient;

  private dataBase: mongodb.Db;

  async getDataBaseCollection(collection: string): Promise<mongodb.Collection<mongodb.Document>> {
    if (!this.client || !this.dataBase) await this.connectToDataBase();
    const collectionDb: mongodb.Collection = this.dataBase.collection(collection);
    return collectionDb;
  }

  async find(collection: string, query: any = {}): Promise<mongodb.Document[]> {
    const collectionDb = await this.getDataBaseCollection(collection);
    return collectionDb.find(query).toArray();
  }

  async insert(collection: string, document: mongodb.Document): Promise<void> {
    const collectionDb = await this.getDataBaseCollection(collection);
    await collectionDb.insertOne(document);
  }

  async insertMany(collection: string, documents: mongodb.Document[]): Promise<void> {
    const collectionDb = await this.getDataBaseCollection(collection);
    await collectionDb.insertMany(documents);
  }

  async replaceOne(collection: string, filter: any, replacement: mongodb.Document): Promise<void> {
    const collectionDb = await this.getDataBaseCollection(collection);
    await collectionDb.replaceOne(filter, replacement);
  }

  async deleteMany(collection: string, filter: any): Promise<void> {
    const collectionDb = await this.getDataBaseCollection(collection);
    await collectionDb.deleteMany(filter);
  }

  async reset(collection: string): Promise<void> {
    const collectionDb = await this.getDataBaseCollection(collection);
    if (await collectionDb.countDocuments() > 0) {
      await collectionDb.drop();
    }
  }

  private async connectToDataBase(): Promise<void> {
    this.client = new mongodb.MongoClient(DB_URL_CONNEXTION);
    await this.client.connect();
    this.dataBase = this.client.db(DB_DATABASE_NAME);
  }
}
